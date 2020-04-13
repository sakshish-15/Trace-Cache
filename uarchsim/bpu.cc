#include <stdio.h>
#include <inttypes.h>
#include <assert.h>
#include "bpu.h"
//#include "parameters.h"

bpu_t::bpu_t(uint64_t instr_per_cycle,				// "n"
	     uint64_t cond_branch_per_cycle,			// "m"
             uint64_t btb_entries,				// total number of entries in the BTB
	     uint64_t btb_assoc,				// set-associativity of the BTB
	     uint64_t cb_pc_length, uint64_t cb_bhr_length,	// gshare cond. br. predictor: pc length (index size), bhr length
	     uint64_t ib_pc_length, uint64_t ib_bhr_length,	// gshare indirect br. predictor: pc length (index size), bhr length
	     uint64_t ras_size,					// # entries in the RAS
	     uint64_t bq_size					// branch queue size (max. number of outstanding branches)
	    ):instr_per_cycle(instr_per_cycle),
	      cond_branch_per_cycle(cond_branch_per_cycle),
	      btb(btb_entries, instr_per_cycle, btb_assoc, cond_branch_per_cycle),	// construct the branch target buffer (btb)
	      cb_index(cb_pc_length, cb_bhr_length),		// construct gshare index function of conditional branch (cb) predictor
              ib_index(ib_pc_length, ib_bhr_length),		// construct gshare index function of indirect branch (ib) predictor
              ras(ras_size),					// construct return address stack (ras)
	      bq(bq_size) {					// construct branch queue (bq)
   // Memory-allocate the conditional branch (cb) prediction table and indirect branch (ib) prediction table.
   cb = new uint64_t[cb_index.table_size()];
   ib = new uint64_t[ib_index.table_size()];

   // This assertion is required because BTB bank selection assumes a power-of-two number of BTB banks.
   assert(IsPow2(instr_per_cycle));

   // The btb output array is statically size-constrained.
   assert(instr_per_cycle <= MAX_BTB_BANKS);

   // Initialize measurements.

   meas_branch_n = 0;	// # branches
   meas_jumpdir_n = 0;	// # jumps, direct
   meas_calldir_n = 0;	// # calls, direct
   meas_jumpind_n = 0;	// # jumps, indirect
   meas_callind_n = 0;	// # calls, indirect
   meas_jumpret_n = 0;	// # jumps, return

   meas_branch_m = 0;	// # mispredicted branches
   meas_jumpind_m = 0;	// # mispredicted jumps, indirect
   meas_callind_m = 0;	// # mispredicted calls, indirect
   meas_jumpret_m = 0;	// # mispredicted jumps, return
}

bpu_t::~bpu_t() {
}

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
uint64_t bpu_t::predict(uint64_t pc, uint64_t pred_tags[], bool &tc_hit, uint64_t &fetch_bundle_length, uint64_t &branch_vector, uint64_t &pred_vector, uint64_t &next_pc) {
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Preliminary step #1:
   // Before pushing the predicted fetch bundle onto the branch queue, we need to record where the
   // branch queue is currently. This recorded "fetch_pred_tag" enables rolling back the branch queue
   // to where it was, prior to predicting the fetch bundle, in the event of a BTB miss.
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////

   uint64_t fetch_pred_tag;	// return value of this function
   bool fetch_pred_tag_phase;

   bq.mark(fetch_pred_tag, fetch_pred_tag_phase);

   // Merge the phase into the tag, so that the user only handles a single number.
   fetch_pred_tag = ((fetch_pred_tag << 1) | (fetch_pred_tag_phase ? 1 : 0));

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Preliminary step #2:
   // Collect information that will be used to get the predictions.
   // This information will go with each entry that gets pushed onto the branch queue,
   // for the predicted fetch bundle.
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////

   uint64_t fetch_pc;
   uint64_t fetch_cb_bhr;
   uint64_t fetch_ib_bhr;
   uint64_t fetch_cb_pos_in_entry;

   fetch_pc = pc;
   fetch_cb_bhr = cb_index.get_bhr();
   fetch_ib_bhr = ib_index.get_bhr();
   fetch_cb_pos_in_entry = 0;

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Search all the structures "in parallel".
   // BTB, conditional branch predictor, indirect branch predictor, and return address stack.
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////

   uint64_t cb_predictions;			  // "m" conditional branch predictions packed into a uint64_t
   uint64_t ib_predicted_target;		  // predicted target from the indirect branch predictor
   uint64_t ras_predicted_target;		  // predicted target from the return address stack (only popped if fetch bundle ends in a return)
   btb_output_t btb_fetch_bundle[MAX_BTB_BANKS];  // BTB's output for the fetch bundle.

   // Get "m" predictions from the conditional branch predictor.
   // "m" two-bit counters are packed into a uint64_t.
   cb_predictions = cb[cb_index.index(pc)];

   // Get a predicted target from the indirect branch predictor.  It is only used if the fetch bundle ends at a jump indirect or call indirect.
   ib_predicted_target = ib[ib_index.index(pc)];

   // Get a predicted target from the return address stack.  This is only a peek: it is popped only if ultimately used.
   ras_predicted_target = ras.peek();

   // Predict a sequential fetch bundle.
   // 1. Its length (fetch_bundle_length, an output of this function).
   // 2. BTB information (hit, type, target) at each slot within the sequential fetch bundle (btb_fetch_bundle[]).
   // 3. Its next pc, if it can be provided by the BTB (next_pc, an output of this function).
   btb.lookup(pc, cb_predictions, fetch_bundle_length, btb_fetch_bundle, next_pc);

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // FIX_ME #TRACECACHE
   //
   // Add your search of the Trace Cache Metadata (TCM) here.
   // It should look like btb.lookup(), and by calling it after btb.lookup(), a trace cache hit will override
   // the fetch_bundle_length, btb_fetch_bundle, and next_pc.
   //
   // tc_hit = tcm.lookup(input:pc, input:cb_predictions, output:fetch_bundle_length, output:btb_fetch_bundle, output:next_pc);
   //
   // 1. Use pc and cb_predictions to search your TCM.
   // 2. The TCM outputs -- fetch_bundle_length, btb_fetch_bundle[], and next_pc -- should only be overwritten
   //    in the case of a trace cache hit.  That is, only overwrite the sequential fetch bundle with
   //    a non-sequential fetch bundle, if the trace cache provided one (trace cache hit).
   // 3. The main loop, below, should work irrespective of the source of the fetch bundle (TCM or BTB).
   //    The trace selection policy MUST be as follows; you may add additional constraints, e.g., related
   //    to the kinds of embedded conditional branches allowed (taken, confident, atomic trace sel., etc.).
   //    - Stop after m'th conditional branch.
   //    - Note: there can be any number of jump directs in the trace.
   //    - Stop after call direct.    (Support one RAS operation per fetch cycle.)
   //    - Stop after jump indirect.  (Don't want indirect targets to be part of the trace cache hit logic.)
   //    - Stop after call indirect.  (Don't want indirect targets to be part of the trace cache hit logic.
   //                                  Support one RAS operation per fetch cycle.)
   //    - Stop after return.         (Don't want indirect targets to be part of the trace cache hit logic.
   //                                  Support one RAS operation per fetch cycle.)
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   // tc_hit = tcm.lookup(pc, cb_predictions, fetch_bundle_length, btb_fetch_bundle, next_pc);
   tc_hit = false;
 
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////
   // 1. Push entries into the branch queue, for all of the branches in the predicted fetch bundle.
   // 2. Update global histories and the RAS.
   // 3. Predict the fetch bundle's next pc if it ends in a jump indirect, call indirect, or return.
   //    The TCM/BTB lookups, above, will have punted on providing a next pc in this case.
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////

   uint64_t b = 0;				// count of branches in the fetch bundle
   bool taken;					// taken/not-taken prediction for the current conditional branch (where we are at in the fetch bundle)
   uint64_t pred_tag;				// pred_tag is the index into the branch queue for the newly pushed branch
   bool pred_tag_phase;				// this will get appended to pred_tag so that the user interacts with the BPU via a single number

   branch_vector = 0; // Initialize this output of the function (locates branches within the fetch bundle)
   pred_vector = 0;   // Initialize this output of the function (taken/not-taken predictions of each conditional branch within the fetch bundle)
   for (uint64_t i = 0; i < fetch_bundle_length; i++) {
      if (btb_fetch_bundle[i].hit) {
         // (trace cache miss) The BTB predicted that this instruction is a branch.
         // (trace cache hit) The TCM says with certainty that this instruction is a branch.

	 // Set the bit, in "branch_vector" (an output of this function), corresponding to this instruction in the fetch bundle.
	 branch_vector |= (1 << i);

	 // Push an entry into the branch queue.
	 // This merely allocates the entry; below, we set the entry's contents.
	 bq.push(pred_tag, pred_tag_phase);

	 // Merge the pred_tag and pred_tag_phase into a single pred_tag,
	 // and output this unified pred_tag to the user via the pred_tags array argument.
	 pred_tags[b] = ((pred_tag << 1) | (pred_tag_phase ? 1 : 0));

	 // Set up context-related fields in the new branch queue entry.
	 bq.bq[pred_tag].branch_type = btb_fetch_bundle[i].branch_type;
	 bq.bq[pred_tag].precise_cb_bhr = cb_index.get_bhr();
	 bq.bq[pred_tag].precise_ib_bhr = ib_index.get_bhr();
	 bq.bq[pred_tag].precise_ras_tos = ras.get_tos();
	 bq.bq[pred_tag].fetch_pc = fetch_pc;
	 bq.bq[pred_tag].fetch_cb_bhr = fetch_cb_bhr;
	 bq.bq[pred_tag].fetch_ib_bhr = fetch_ib_bhr;
	 bq.bq[pred_tag].fetch_cb_pos_in_entry = 0;   // Only relevant for conditional branches, so it may be other than 0 for them.

	 // Initialize the misp. flag to indicate, as far as we know at this point, the branch is not mispredicted.
	 bq.bq[pred_tag].misp = false;

	 // Take action according to the branch type.
	 switch (btb_fetch_bundle[i].branch_type) {
	    case BTB_BRANCH:
	       // The low two bits of cb_predictions correspond to the next two-bit counter to examine (because we shift it right, subsequently).
	       // From this two-bit counter, set the taken flag, accordingly.
	       taken = ((cb_predictions & 3) >= 2);

	       // Shift out the used-up 2-bit counter, to set up the next conditional branch.
	       cb_predictions = (cb_predictions >> 2);

	       if (taken) {
	          // Set the taken/not-taken bit, in "pred_vector" (an output of this function), corresponding to this instruction in the fetch bundle.
	          pred_vector |= (1 << i);
	       }

	       // Record the prediction.
	       bq.bq[pred_tag].taken = taken;
	       bq.bq[pred_tag].next_pc = (taken ? btb_fetch_bundle[i].target : (fetch_pc + ((i + 1) << 2)));

	       // Record this conditional branch's position within the conditional branch prediction bundle.
	       bq.bq[pred_tag].fetch_cb_pos_in_entry = fetch_cb_pos_in_entry;

	       // Increment the position to set up for the next conditional branch in the conditional branch prediction bundle.
	       fetch_cb_pos_in_entry++;

	       // Update the BHRs of the conditional branch predictor and indirect branch predictor.
	       cb_index.update_bhr(taken);
	       ib_index.update_bhr(taken);
	       break;

	    case BTB_JUMP_DIRECT:
	       // Record the prediction.
	       bq.bq[pred_tag].taken = true;
	       bq.bq[pred_tag].next_pc = btb_fetch_bundle[i].target;
	       break;

	    case BTB_CALL_DIRECT:
	       // We want to model only one RAS operation per fetch cycle, so assert that the call is the last instruction in the fetch bundle.
	       assert(i == (fetch_bundle_length - 1));

	       // Push the RAS with the PC of the call instruction plus 4 (next sequential PC).
	       // The calculation is shown below: fetch_PC + (# instructions in fetch bundle before the call + 1 instruction for the call)*4.
	       ras.push(fetch_pc + ((i + 1) << 2));

	       // Record the prediction.
	       bq.bq[pred_tag].taken = true;
	       bq.bq[pred_tag].next_pc = btb_fetch_bundle[i].target;
	       break;

	    case BTB_JUMP_INDIRECT:
	       // Indirect branches terminate a fetch bundle irrespective of the source, TCM or BTB.
	       assert(i == (fetch_bundle_length - 1));

	       // Neither the TCM nor the BTB were able to provide the next_pc.  Predict it here.
	       next_pc = ib_predicted_target;

	       // Record the prediction.
	       bq.bq[pred_tag].taken = true;
	       bq.bq[pred_tag].next_pc = ib_predicted_target;
	       break;

	    case BTB_CALL_INDIRECT:
	       // Indirect branches terminate a fetch bundle irrespective of the source, TCM or BTB.
	       // Moreover, we want to model only one RAS operation per fetch cycle, so assert that the call is the last instruction in the fetch bundle.
	       assert(i == (fetch_bundle_length - 1));

	       // Neither the TCM nor the BTB were able to provide the next_pc.  Predict it here.
	       next_pc = ib_predicted_target;

	       // Push the RAS with the PC of the call instruction plus 4 (next sequential PC).
	       // The calculation is shown below: fetch_PC + (# instructions in fetch bundle before the call + 1 instruction for the call)*4.
	       ras.push(fetch_pc + ((i + 1) << 2));

	       // Record the prediction.
	       bq.bq[pred_tag].taken = true;
	       bq.bq[pred_tag].next_pc = ib_predicted_target;
	       break;

	    case BTB_RETURN:
	       // Indirect branches terminate a fetch bundle irrespective of the source, TCM or BTB.
	       // Moreover, we want to model only one RAS operation per fetch cycle, so assert that the return is the last instruction in the fetch bundle.
	       assert(i == (fetch_bundle_length - 1));

	       // Neither the TCM nor the BTB were able to provide the next_pc.  Predict it here.
	       next_pc = ras_predicted_target;

	       // Pop the RAS.
	       assert(next_pc == ras.pop());

	       // Record the prediction.
	       bq.bq[pred_tag].taken = true;
	       bq.bq[pred_tag].next_pc = ras_predicted_target;
	       break;

	    default:
	       assert(0);
	       break;
         }
	 b++;	// increment number of branches
      }
   }

   return(fetch_pred_tag);
}


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
void bpu_t::btb_miss(uint64_t fetch_pred_tag, uint64_t pc, uint64_t btb_miss_bit, uint64_t btb_miss_target, insn_t insn) {

   // Extract the pred_tag and pred_tag_phase from the unified fetch_pred_tag.

   uint64_t pred_tag = (fetch_pred_tag >> 1);
   bool pred_tag_phase = (((fetch_pred_tag & 1) == 1) ? true : false);

   // 1. Roll-back the branch queue to where it was prior to predicting the fetch bundle (fetch_pred_tag).

   bq.rollback(pred_tag, pred_tag_phase, false);

   // 2. Restore checkpointed global histories and the RAS.

   cb_index.set_bhr(bq.bq[pred_tag].precise_cb_bhr);
   ib_index.set_bhr(bq.bq[pred_tag].precise_ib_bhr);
   ras.set_tos(bq.bq[pred_tag].precise_ras_tos);

   // 3. Add the missing branch to the BTB.

   btb.update(pc, btb_miss_bit, btb_miss_target, insn);
}


// A mispredicted branch was detected.
// 1. Roll-back the branch queue to the mispredicted branch's entry.
// 2. Correct the mispredicted branch's information in its branch queue entry.
// 3. Restore checkpointed global histories and the RAS (as best we can for RAS).
// 4. Note that the branch was mispredicted (for measuring mispredictions at retirement).
void bpu_t::mispredict(uint64_t branch_pred_tag, bool taken, uint64_t next_pc) {
   // Extract the pred_tag and pred_tag_phase from the unified branch_pred_tag.

   uint64_t pred_tag = (branch_pred_tag >> 1);
   bool pred_tag_phase = (((branch_pred_tag & 1) == 1) ? true : false);

   // 1. Roll-back the branch queue to the mispredicted branch's entry.
   //    Then push the branch back onto it.

   bq.rollback(pred_tag, pred_tag_phase, true);

   uint64_t temp_pred_tag;
   bool temp_pred_tag_phase;
   bq.push(temp_pred_tag, temp_pred_tag_phase);	// Need to push the branch back onto the branch queue.
   assert((temp_pred_tag == pred_tag) && (temp_pred_tag_phase == pred_tag_phase));

   // 2. Correct the mispredicted branch's information in its branch queue entry.

   assert(bq.bq[pred_tag].next_pc != next_pc);
   bq.bq[pred_tag].next_pc = next_pc;

   if (bq.bq[pred_tag].branch_type == BTB_BRANCH) {
      assert(bq.bq[pred_tag].taken != taken);
      bq.bq[pred_tag].taken = taken;
   }
   else {
      assert(taken);
      assert(bq.bq[pred_tag].taken);
   }
 
   // 3. Restore checkpointed global histories and the RAS (as best we can for RAS).

   cb_index.set_bhr(bq.bq[pred_tag].precise_cb_bhr);
   ib_index.set_bhr(bq.bq[pred_tag].precise_ib_bhr);
   ras.set_tos(bq.bq[pred_tag].precise_ras_tos);

   // 4. Note that the branch was mispredicted (for measuring mispredictions at retirement).

   bq.bq[pred_tag].misp = true;
}


// Commit the indicated branch from the branch queue.
// We assert that it is at the head.
void bpu_t::commit(uint64_t branch_pred_tag) {
   // Pop the branch queue. It returns the pred_tag/pred_tag_phase of the head entry prior to popping it.
   uint64_t pred_tag;
   bool pred_tag_phase;
   bq.pop(pred_tag, pred_tag_phase);

   // Assert that the branch_pred_tag (pred_tag of the branch being committed from the pipeline) corresponds to the popped branch queue entry.
   assert(branch_pred_tag == ((pred_tag << 1) | (pred_tag_phase ? 1 : 0)));

   // Update the conditional branch predictor or indirect branch predictor.
   // Update measurements.
   uint64_t *cb_counters;	// FYI: The compiler forbids declaring these four local variables inside "case BTB_BRANCH:".
   uint64_t shamt;
   uint64_t mask;
   uint64_t ctr;
   switch (bq.bq[pred_tag].branch_type) {
      case BTB_BRANCH:
	 // Re-reference the conditional branch predictor, using the same context that was used by
	 // the fetch bundle that this branch was a part of.
         // Using this original context, we re-reference the same "m" counters from the conditional branch predictor.
         // "m" two-bit counters are packed into a uint64_t.
	 cb_counters = &(  cb[ cb_index.index(bq.bq[pred_tag].fetch_pc, bq.bq[pred_tag].fetch_cb_bhr) ]  );

	 // Prepare for reading and writing the 2-bit counter that was used to predict this branch.
	 // We need a shift-amount ("shamt") and a mask ("mask") that can be used to read/write just that counter.
	 // "shamt" = the branch's position in the entry times 2, for 2-bit counters.
	 // "mask" = (3 << shamt).
	 shamt = (bq.bq[pred_tag].fetch_cb_pos_in_entry << 1);     
	 mask = (3 << shamt);

	 // Extract a local copy of the 2-bit counter that was used to predict this branch.
	 ctr = (((*cb_counters) & mask) >> shamt);

	 // Increment or decrement the local copy of the 2-bit counter, based on the branch's outcome.
	 if (bq.bq[pred_tag].taken) {
	    if (ctr < 3)
	       ctr++;
	 }
	 else {
	    if (ctr > 0)
	       ctr--;
	 }

	 // Write the modified local copy of the 2-bit counter back into the predictor's entry.
	 *cb_counters = (((*cb_counters) & (~mask)) | (ctr << shamt));

	 // Update measurements.
	 meas_branch_n++;
	 if (bq.bq[pred_tag].misp)
	    meas_branch_m++;
         break;

      case BTB_JUMP_DIRECT:
	 // Update measurements.
	 meas_jumpdir_n++;
	 assert(!bq.bq[pred_tag].misp);
         break;

      case BTB_CALL_DIRECT:
	 // Update measurements.
	 meas_calldir_n++;
	 assert(!bq.bq[pred_tag].misp);
         break;

      case BTB_JUMP_INDIRECT:
      case BTB_CALL_INDIRECT:
	 // Re-reference the indirect branch predictor, using the same context that was used by
	 // the fetch bundle that this branch was a part of.
	 ib[ ib_index.index(bq.bq[pred_tag].fetch_pc, bq.bq[pred_tag].fetch_ib_bhr) ] = bq.bq[pred_tag].next_pc;

	 // Update measurements.
	 if (bq.bq[pred_tag].branch_type == BTB_JUMP_INDIRECT) {
	    meas_jumpind_n++;
	    if (bq.bq[pred_tag].misp)
	       meas_jumpind_m++;
	 }
	 else {
	    meas_callind_n++;
	    if (bq.bq[pred_tag].misp)
	       meas_callind_m++;
	 }
         break;

      case BTB_RETURN:
         // Update measurements.
	 meas_jumpret_n++;
	 if (bq.bq[pred_tag].misp)
	    meas_jumpret_m++;
         break;

      default:
	 assert(0);
         break;
   }
}


// Complete squash.
// 1. Roll-back the branch queue to the head entry.
// 2. Restore checkpointed global histories and the RAS (as best we can for RAS).
void bpu_t::flush() {
   uint64_t pred_tag;

   // 1. Roll-back the branch queue to the head entry.
   pred_tag = bq.flush();  // "pred_tag" is the index of the head entry.

   // 2. Restore checkpointed global histories and the RAS (as best we can for RAS).
   cb_index.set_bhr(bq.bq[pred_tag].precise_cb_bhr);
   ib_index.set_bhr(bq.bq[pred_tag].precise_ib_bhr);
   ras.set_tos(bq.bq[pred_tag].precise_ras_tos);
}


// Output all branch prediction measurements.

#define BP_OUTPUT(fp, str, n, m, i) \
	fprintf((fp), "%s%10lu %10lu %5.2lf%% %5.2lf\n", (str), (n), (m), 100.0*((double)(m)/(double)(n)), 1000.0*((double)(m)/(double)(i)))

void bpu_t::output(uint64_t num_instr, FILE *fp) {
   uint64_t all = (meas_branch_n + meas_jumpdir_n + meas_calldir_n + meas_jumpind_n + meas_callind_n + meas_jumpret_n);
   uint64_t all_misp = (meas_branch_m + meas_jumpind_m + meas_callind_m + meas_jumpret_m);
   printf("BRANCH PREDICTION MEASUREMENTS---------------------\n");
   printf("Type                      n          m     mr  mpki\n");
   BP_OUTPUT(fp, "All              ", all, all_misp, num_instr);
   BP_OUTPUT(fp, "Branch           ", meas_branch_n, meas_branch_m, num_instr);
   BP_OUTPUT(fp, "Jump Direct      ", meas_jumpdir_n, (uint64_t)0, num_instr);
   BP_OUTPUT(fp, "Call Direct      ", meas_calldir_n, (uint64_t)0, num_instr);
   BP_OUTPUT(fp, "Jump Indirect    ", meas_jumpind_n, meas_jumpind_m, num_instr);
   BP_OUTPUT(fp, "Call Indirect    ", meas_callind_n, meas_callind_m, num_instr);
   BP_OUTPUT(fp, "Return           ", meas_jumpret_n, meas_jumpret_m, num_instr);
}

