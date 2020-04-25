#include "pipeline.h"
#include "mmu.h"
#include "CacheClass.h"


void pipeline_t::fetch() {
   // Variables related to instruction cache.
   unsigned int line1;
   unsigned int line2;
   bool hit1;
   bool hit2;
   cycle_t resolve_cycle1;
   cycle_t resolve_cycle2;

   // Variables influencing when to terminate fetch bundle.
   unsigned int i;	// iterate up to fetch width
   //bool stop;		// branch, icache block boundary, etc.

   // Instruction fetched from mmu.
   insn_t insn;		// "insn" is used by some MACROs, hence, need to use this name for the instruction variable.
   reg_t trap_cause = 0;
 
   // PAY index for newly fetched instruction.
   unsigned int index;

   // Link to corresponding instrution in functional simulator, i.e., map_to_actual() functionality.
   db_t *actual;
 
   // Signals from Branch Prediction Unit (BPU).
   uint64_t fetch_pred_tag;
   uint64_t pred_tags[MAX_BTB_BANKS];
   bool tc_hit;
   uint64_t fetch_bundle_length;
   uint64_t branch_vector;
   uint64_t pred_vector;
   uint64_t next_fetch_pc;
 
   // Miscellaneous control signals.
   bool pred_valid;
   unsigned int pay_checkpoint;
   unsigned int direct_target;
   uint64_t next_pc;
   uint64_t pred_tag;
   bool btb_miss;
   uint64_t btb_miss_bit;
   unsigned int btb_miss_target;
   insn_t btb_miss_insn;
   uint64_t next_pred_tag_index;
   uint64_t save_pc;

   /////////////////////////////
   // Stall logic.
   /////////////////////////////

   // Stall the Fetch Stage if either:
   // 1. The Decode Stage is stalled.
   // 2. An I$ miss has not yet resolved.
   if ((DECODE[0].valid) ||		// Decode Stage is stalled.
       (cycle < next_fetch_cycle)) {	// I$ miss has not yet resolved.
      return;
   }

   /////////////////////////////
   // Model I$ misses.
   /////////////////////////////

   if (!PERFECT_ICACHE) {
      line1 = (pc >> L1_IC_LINE_SIZE);
      resolve_cycle1 = IC->Access(Tid, cycle, (line1 << L1_IC_LINE_SIZE), false, &hit1);
      if (IC_INTERLEAVED) {
         // Access next consecutive line.
         line2 = (pc >> L1_IC_LINE_SIZE) + 1;
         resolve_cycle2 = IC->Access(Tid, cycle, (line2 << L1_IC_LINE_SIZE), false, &hit2);
      }
      else {
         hit2 = true;
      }

      if (!hit1 || !hit2) {
         next_fetch_cycle = MAX((hit1 ? 0 : resolve_cycle1), (hit2 ? 0 : resolve_cycle2));
         assert(next_fetch_cycle > cycle);
         return;
      }
   }

   /////////////////////////////
   // Access the branch prediction unit (BPU).
   /////////////////////////////
   if (PERFECT_BRANCH_PRED) {
      pred_valid = false;
      fetch_bundle_length = fetch_width;
   }
   else {
      pred_valid = true;
      fetch_pred_tag = BPU.predict(pc, pred_tags, tc_hit, fetch_bundle_length, branch_vector, pred_vector, next_fetch_pc);
      assert(fetch_bundle_length <= fetch_width);

      // Checkpoint PAY, so that we can squash the fetch bundle and repredict it if BTB hits are flawed.
      pay_checkpoint = PAY.checkpoint();
   }

//
// TODO: PERFECT_BRANCH_PREDICTION still works, but it also implements PERFECT_FETCH implicitly. Must decouple the two.
// TODO: PERFECT_FETCH itself needs to be implemented again.
// TODO: BPU needs to also reference IC_INTERLEAVED when forming its fetch bundle prediction.
//

   /////////////////////////////
   // Compose fetch bundle.
   /////////////////////////////

   i = 0;
   btb_miss = false;
   next_pred_tag_index = 0;
   save_pc = pc;
   while ((i < fetch_bundle_length) && (!btb_miss)) {

      //////////////////////////////////////////////////////
      // Fetch instruction -or- inject NOP for fetch stall.
      //////////////////////////////////////////////////////

      // Try fetching the instruction via the MMU.
      // Generate a "NOP with fetch exception" if the MMU reference generates an exception.
      fetch_exception = false;
      try {
         insn = (mmu->load_insn(pc)).insn;
      }
      catch (trap_t& t) {
         insn = insn_t(INSN_NOP);
	 fetch_exception = true;
         trap_cause = t.cause();
      }

      // Put the instruction's information into PAY.
      index = PAY.push();
      PAY.buf[index].inst = insn;
      PAY.buf[index].pc = pc;
      PAY.buf[index].sequence = sequence;
      PAY.buf[index].fetch_exception = fetch_exception;
      PAY.buf[index].fetch_exception_cause = trap_cause;

      //////////////////////////////////////////////////////
      // map_to_actual()
      //////////////////////////////////////////////////////

      // Try to link the instruction to the corresponding instruction in the functional simulator.
      // NOTE: Even when NOPs are injected, successfully mapping to actual is not a problem,
      // as the NOP instructions will never be committed.
      PAY.map_to_actual(this, index, Tid);
      if (PAY.buf[index].good_instruction)
         actual = pipe->peek(PAY.buf[index].db_index);
      else
         actual = (db_t *) NULL;

      //////////////////////////////////////////////////////
      // Set next_pc and the prediction tag.
      //////////////////////////////////////////////////////

      switch (insn.opcode()) {
         case OP_JAL:
            direct_target = JUMP_TARGET;
	    if (PERFECT_BRANCH_PRED) {
               next_pc = (actual ? actual->a_next_pc : direct_target);
	       pred_tag = 0;
	    }
	    else {
	       assert(pred_valid);
	       if (tc_hit) {
		  // Trace Cache hit.
		  // Trace Cache should know exactly where all the branches are in the fetch bundle.
	          assert(branch_vector & (1 << i));
		  if (i == (fetch_bundle_length - 1)) {
		     // This is the last instruction in the fetch bundle.
		     // Assert that the BPU's predicted next fetch pc equals the jump's direct target.
		     assert(next_fetch_pc == direct_target);
		  }
		  next_pc = direct_target;
	          pred_tag = pred_tags[next_pred_tag_index];
		  next_pred_tag_index++;
	       }
	       else if (branch_vector & (1 << i)) {
		  // Trace Cache miss, BTB hit.
		  // BTB properly detected this jump instruction.  Therefore:
		  // 1. Assert that the BPU said this jump terminated the fetch bundle.
		  // 2. Assert that the BPU's predicted next fetch pc equals the jump's direct target.
	          assert(i == (fetch_bundle_length - 1));
	          assert(next_fetch_pc == direct_target);
		  next_pc = direct_target;
	          pred_tag = pred_tags[next_pred_tag_index];
		  next_pred_tag_index++;
	       }
	       else {
		  // Trace Cache miss, BTB miss.
	          btb_miss = true;
	          btb_miss_bit = i;
		  btb_miss_target = direct_target;
		  btb_miss_insn = insn;
	       }
	    }
            break;

         case OP_JALR:
	    if (PERFECT_BRANCH_PRED) {
               next_pc = (actual ? actual->a_next_pc : INCREMENT_PC(pc));
	       pred_tag = 0;
	    }
	    else {
	       assert(pred_valid);
	       if (tc_hit) {
		  // Trace Cache hit.
		  // Trace Cache should know exactly where all the branches are in the fetch bundle.
	          assert(branch_vector & (1 << i));
		  // Assert that the BPU said this jump indirect terminated the fetch bundle.
	          assert(i == (fetch_bundle_length - 1));
	          next_pc = next_fetch_pc;
	          pred_tag = pred_tags[next_pred_tag_index];
		  next_pred_tag_index++;
	       }
	       else if (branch_vector & (1 << i)) {
		  // Trace Cache miss, BTB hit.
		  // BTB properly detected this jump indirect instruction.  Therefore:
		  // Assert that the BPU said this jump indirect terminated the fetch bundle.
	          assert(i == (fetch_bundle_length - 1));
	          next_pc = next_fetch_pc;
	          pred_tag = pred_tags[next_pred_tag_index];
		  next_pred_tag_index++;
	       }
	       else {
		  // Trace Cache miss, BTB miss.
	          btb_miss = true;
	          btb_miss_bit = i;
		  btb_miss_target = 0;
		  btb_miss_insn = insn;
	       }
	    }
            break;

         case OP_BRANCH:
            direct_target = BRANCH_TARGET;
	    if (PERFECT_BRANCH_PRED) {
               next_pc = (actual ? actual->a_next_pc : INCREMENT_PC(pc));
	       pred_tag = 0;
	    }
	    else {
	       assert(pred_valid);
	       if (tc_hit) {
		  // Trace Cache hit.
		  // Trace Cache should know exactly where all the branches are in the fetch bundle.
	          assert(branch_vector & (1 << i));

		  // Use pred_vector to select between taken and not-taken targets.
		  next_pc = ((pred_vector & (1 << i)) ? direct_target : INCREMENT_PC(pc));
	          pred_tag = pred_tags[next_pred_tag_index];
		  next_pred_tag_index++;

		  // If the branch is the last instruction in the fetch bundle,
		  // check that the BPU is consistent between its pred_vector and next_fetch_pc.
	          if (i == (fetch_bundle_length - 1))
		     assert(next_pc == next_fetch_pc);
	       }
	       else if (branch_vector & (1 << i)) {
		  // Trace Cache miss, BTB hit.
		  // BTB properly detected this branch instruction.

		  // Use pred_vector to select between taken and not-taken targets.
		  next_pc = ((pred_vector & (1 << i)) ? direct_target : INCREMENT_PC(pc));
	          pred_tag = pred_tags[next_pred_tag_index];
		  next_pred_tag_index++;

		  // 1. If the branch is the last instruction in the fetch bundle,
		  //    check that the BPU is consistent between its pred_vector and next_fetch_pc.
		  // 2. If the branch is NOT the last instruction in the fetch bundle,
		  //    assert that the branch is predicted not-taken: it has to be a sequential fetch bundle.
	          if (i == (fetch_bundle_length - 1))
		     assert(next_pc == next_fetch_pc);     // 1.
	          else
		     assert(next_pc == INCREMENT_PC(pc));  // 2.
	       }
	       else {
		  // Trace Cache miss, BTB miss.
	          btb_miss = true;
	          btb_miss_bit = i;
		  btb_miss_target = direct_target;
		  btb_miss_insn = insn;
	       }
	    }
            break;

         default:
	    if (pred_valid) {
	       // Assert that neither the Trace Cache nor the BTB presumed this instruction to be a branch.
	       assert((branch_vector & (1 << i)) == 0);

	       // If this is the last instruction in the fetch bundle:
	       // 1. Assert the BPU's predicted fetch bundle is of maximum width.
	       // 2. Assert the BPU's predicted next fetch pc is sequential.
	       if (i == (fetch_bundle_length - 1)) {
	          assert(fetch_bundle_length == fetch_width);
		  assert(next_fetch_pc == INCREMENT_PC(pc));
	       }
	    }
            next_pc = INCREMENT_PC(pc);
            break;
      }

      // Set payload buffer entry's next_pc and pred_tag.
      PAY.buf[index].next_pc = next_pc;
      PAY.buf[index].pred_tag = pred_tag;

      // Latch instruction into fetch-decode pipeline register.
      DECODE[i].valid = true;
      DECODE[i].index = index;

      // Keep count of number of fetched instructions.
      i++;

#if 0  // IC_INTERLEAVED broken: always assumes interleaved I$.
      // If not already stopped:
      // Stop if the I$ is not interleaved and if a line boundary is crossed.
      if (!stop && !IC_INTERLEAVED) {
         line1 = (pc >> L1_IC_LINE_SIZE);
         line2 = (next_pc >> L1_IC_LINE_SIZE);
         stop = (line1 != line2);
      }
#endif

      // Go to next PC.
      pc = next_pc;
      state.pc = pc;
      sequence++;
   }			// while()


   // BTB miss recovery.
   if (pred_valid) {
      if (btb_miss) {
         // BTB misses are not checked when there is a trace cache hit.
	 // Therefore, assert the trace cache missed.
	 assert(!tc_hit);

         // Restore the saved pc.
	 pc = save_pc;
	 state.pc = pc;

         // BPU:
	 // 1. Update BTB with the missed branch.
	 // 2. Rollback BPU to re-predict the same (but more accurate) fetch bundle in the next fetch cycle.
	 BPU.btb_miss(fetch_pred_tag, pc, btb_miss_bit, btb_miss_target, btb_miss_insn);

         // Restore PAY.
	 PAY.restore(pay_checkpoint);

	 // Squash malformed fetch bundle in fetch-decode pipeline register.
         DECODE[0].valid = false;
      }
      else {
         assert(pc == next_fetch_pc);
      }
      BPU.trace_constructor(!btb_miss, tc_hit); //call trace cache line fill buffer
   }
}			// fetch()
