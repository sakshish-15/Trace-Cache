#include "btb.h"

#define MAX_TCM_BUNDLE 16


// A TCM entry
typedef
struct {
   // Metadata for hit/miss determination and replacement.
   bool valid;
   uint64_t tag;
   uint64_t lru;
   uint64_t br_flags;
   uint64_t br_mask;

   // Payload.
   uint64_t tcm_bundle_length; //length of the trace cache bundle
   btb_output_t tcm_fetch_bundle[MAX_TCM_BUNDLE]; // TCM's output for the fetch bundle.
   uint64_t next_pc; //equal to pc for the next fetch bundle
} tcm_entry_t;


class tcm_t {
private:
	// The trace cache has 2 dimensions: number of sets and number of ways per set (associativity)
	tcm_entry_t **tcm;
	uint64_t sets;
	uint64_t assoc;

	uint64_t log2sets;  // number of pc bits that selects the set within a bank

	uint64_t pc_line_fill;
	uint64_t cb_predictions_line_fill;
	uint64_t br_cntr_line_fill;
	uint64_t cond_branch_line_fill; // "m": maximum number of conditional branches in the line fill.
	uint64_t br_mask_line_fill;
	uint64_t valid_line_fill; //line fill buffer has a valid entry in it
	uint64_t index_line_fill;
	uint64_t tag_line_fill;
	uint64_t line_fill_full; // assert when m branches or max instructions present in line fill, deassert when commit or rollback
	tcm_entry_t line_fill_buffer_entry;

	//line fill buffer checkpoint
	uint64_t valid;
	uint64_t cond_branch;
	uint64_t br_mask;
	uint64_t br_cntr;
	tcm_entry_t checkpoint_line;


	////////////////////////////////////
	// Private utility functions.
	// Comments are in tcm.cc.
	////////////////////////////////////

	bool search(uint64_t pc, uint64_t cb_predictions, uint64_t &set, uint64_t &way);
	void update_lru(uint64_t set, uint64_t way);
	btb_branch_type_e decode(insn_t insn);
	void checkpoint_line_fill();
	

public:
	tcm_t(uint64_t num_entries, uint64_t assoc, uint64_t cond_branch_per_cycle);
	~tcm_t();
	// Search the TCM for a hit
	// Inputs: pc: PC for the first instruction of the bundle
	//         cb_predictions: "m" predictions from branch predictor
	// Outputs:returns true if TCM hit else returns false
	// 	   tcm_bundle_length: length of the bundle
	// 	   tcm_detch_bundle: the non sequential fetch bundle
	// 	   next_pc: the taken or fall through pc
	bool lookup(uint64_t pc, uint64_t cb_predictions, uint64_t &tcm_bundle_length, btb_output_t tcm_fetch_bundle[], uint64_t &tcm_next_pc);
	// Called after btb is hit and fetch bundle is formed
	void line_fill_buf(uint64_t pc, uint64_t cb_predictions, uint64_t fetch_bundle_length, btb_output_t btb_fetch_bundle[], uint64_t next_pc);//TODO assert while filling line, last_pc == next_pc
	// At btb_miss, clear the last entry pushed in line fill buffer
	void rollback_line_fill();
	// At the end of a properly formed fetch bundle, 
	// commit the last entry pushed in line fill buffer
	void commit_line_fill();
};
