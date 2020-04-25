
#include "processor.h"
#include "decode.h"
#include "config.h"

#include "tcm.h"
#include "bq.h"
#include "gshare.h"
#include "ras.h"


class bpu_t {
private:
	// Fetch bundle constraints.
	uint64_t instr_per_cycle;
	uint64_t cond_branch_per_cycle;

	// Branch queue for keeping track of all outstanding branch predictions.
	bq_t bq;

	// Branch Target Buffer (BTB):
	//
	// Locates branches within a sequential fetch bundle, and provides their types and
	// taken targets (latter for just conditional branches and direct jumps).
	//
	// The BTB's information, along with conditional branch T/NT predictions, is critical
	// for determining the fetch bundle's length and for selecting the PC of the next
	// fetch bundle among multiple choices.
	btb_t btb;
	
	tcm_t tcm;
	uint64_t tc_hit_cnt; // stats counter
	uint64_t tc_diff_bun; // stats counter
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIX_ME #TRACECACHE
	//
	// Trace Cache Metadata (TCM):
	//
	// If you think about it, a key role of the trace cache is that of a "BTB alternate"
	// for non-sequential fetch bundles.  A T$ hit overrides the BTB and makes the
	// "BTB alternate" take effect for the current fetch bundle.
	//
	// To make the "BTB alternate" analogy clear, we restate the text from the BTB comments section above,
	// and simply replace "BTB's" with "TCM's":
	// "The TCM's information, along with conditional branch T/NT predictions, is critical
	// for determining the fetch bundle's length and for selecting the PC of the next
	// fetch bundle among multiple choices."
	// Note that the TCM will record another piece of metadata, the trace's length.
	// This is a requirement omitted in the MICRO-29 paper (although one could alternatively assume
	// padding with NOPs; however this BPU interface must return a fetch bundle length).
	//
	// The trace cache's other key role is supplying the instructions of the non-sequential fetch bundle,
	// but for simulator efficiency, you don't actually need to have a structure for the instruction trace.
	// I just retooled the existing instruction fetching code in fetch.cc which grabs the instructions
	// from the memory management unit (essentially from the memory image of the program binary).
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// tcm_t tcm;   <----

	// Gshare predictor for conditional branches.
	uint64_t *cb;
	gshare_index_t cb_index;

	// Gshare predictor for indirect branches.
	uint64_t *ib;
	gshare_index_t ib_index;

	// Return address stack for predicting return targets.
	ras_t ras;

	// Measurements.
	uint64_t meas_branch_n;		// # branches
	uint64_t meas_jumpdir_n;	// # jumps, direct
	uint64_t meas_calldir_n;	// # calls, direct
	uint64_t meas_jumpind_n;	// # jumps, indirect
	uint64_t meas_callind_n;	// # calls, indirect
	uint64_t meas_jumpret_n;	// # jumps, return

	uint64_t meas_branch_m;		// # mispredicted branches
	uint64_t meas_jumpind_m;	// # mispredicted jumps, indirect
	uint64_t meas_callind_m;	// # mispredicted calls, indirect
	uint64_t meas_jumpret_m;	// # mispredicted jumps, return
	
	uint64_t lfb_pc;
	uint64_t lfb_cb_predictions;
	uint64_t lfb_fetch_bundle_length;
	uint64_t lfb_next_pc;
	btb_output_t lfb_fetch_bundle[MAX_BTB_BANKS];

public:
	bpu_t(uint64_t instr_per_cycle,				// "n"
	      uint64_t cond_branch_per_cycle,			// "m"
	      uint64_t btb_entries,				// total number of entries in the BTB
	      uint64_t btb_assoc,				// set-associativity of the BTB
	      uint64_t cb_pc_length, uint64_t cb_bhr_length,	// gshare cond. br. predictor: pc length (index size), bhr length
	      uint64_t ib_pc_length, uint64_t ib_bhr_length,	// gshare indirect br. predictor: pc length (index size), bhr length
	      uint64_t ras_size,				// # entries in the RAS
	      uint64_t bq_size);				// branch queue size (max. number of outstanding branches)
	~bpu_t();

	// Predict the fetch bundle starting at "pc".
	// Inputs:
	//    1. Start PC of fetch bundle (pc).
	// Outputs:
	//    1. Overall fetch bundle prediction tag (return value of function).
	//       Needed for undoing predicted fetch bundle in the branch queue (bq) in case of a BTB miss (which implies a Trace Cache miss).
	//    2. Prediction tags for up to "m" branches (pred_tags).
	//    3. Trace cache hit signal (tc_hit).
	//       If true, the trace cache is supplying information about the non-sequential fetch bundle.
	//       If false, the BTB is supplying information about the sequential fetch bundle.
	//    4. Predicted fetch bundle length (fetch_bundle_length).
	//    5. Bit vector of branches within the fetch bundle (branch_vector).
	//    6. Bit vector of conditional branch predictions within the fetch bundle (pred_vector).
	//    7. Predicted next PC (next_pc).
        uint64_t predict(uint64_t pc, uint64_t pred_tags[], bool &tc_hit, uint64_t &fetch_bundle_length, uint64_t &branch_vector, uint64_t &pred_vector, uint64_t &next_pc);

	// A BTB miss was detected in the predicted fetch bundle.
	// 1. Roll-back the branch queue to where it was prior to predicting the fetch bundle (fetch_pred_tag).
	// 2. Restore checkpointed global histories and the RAS.
	// 3. Add the missing branch to the BTB.
	//
	// FYI:
	// Our low-design-complexity model is that the fetch unit discards the entire predicted fetch bundle and retries
	// until there are no more BTB misses in the predicted fetch bundle.  Only the oldest BTB miss is reported at each iteration so
	// there could be multiple retries in sequence.  Deadlock should not be possible, due to ping-ponging BTB replacements, owing to
	// each instruction slot having a dedicated BTB bank.
	void btb_miss(uint64_t fetch_pred_tag, uint64_t pc, uint64_t btb_miss_bit, uint64_t btb_miss_target, insn_t insn);

	// A mispredicted branch was detected.
	// 1. Roll-back the branch queue to the mispredicted branch's entry.
	// 2. Correct the mispredicted branch's information in its branch queue entry.
	// 3. Restore checkpointed global histories and the RAS (as best we can for RAS).
	// 4. Note that the branch was mispredicted (for measuring mispredictions at retirement).
	void mispredict(uint64_t branch_pred_tag, bool taken, uint64_t next_pc);

	// Commit the indicated branch from the branch queue.
	// We assert that it is at the head.
	void commit(uint64_t branch_pred_tag);

	// Complete squash.
	// 1. Roll-back the branch queue to the head entry.
	// 2. Restore checkpointed global histories and the RAS (as best we can for RAS).
	void flush();
	void trace_constructor (bool valid_fetch_bundle, bool tcm_hit);
	// Output all branch prediction measurements.
	void output(uint64_t num_instr, FILE *fp);
};
