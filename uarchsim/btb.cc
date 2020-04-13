#include <cinttypes>
#include <cassert>
#include <cmath>

#include "processor.h"
#include "decode.h"
#include "config.h"

#include "btb.h"


btb_t::btb_t(uint64_t num_entries, uint64_t banks, uint64_t assoc, uint64_t cond_branch_per_cycle) {
   this->banks = banks;
   this->sets = (num_entries/(banks*assoc));
   this->assoc = assoc;
   this->cond_branch_per_cycle = cond_branch_per_cycle;

   assert(IsPow2(banks));
   assert(IsPow2(sets));

   log2banks = (uint64_t) log2((double)banks);
   log2sets = (uint64_t) log2((double)sets);

   // Allocate the 3D array.
   btb = new btb_entry_t **[banks];
   for (uint64_t b = 0; b < banks; b++) {
      btb[b] = new btb_entry_t *[sets];
      for (uint64_t s = 0; s < sets; s++) {
         btb[b][s] = new btb_entry_t[assoc];
	 for (uint64_t way = 0; way < assoc; way++) {
	    btb[b][s][way].valid = false;
	    btb[b][s][way].lru = way;
	 }
      }
   }
}


btb_t::~btb_t() {
}


//
// pc: The start pc of the fetch bundle.
// cb_predictions: A uint64_t packed with "m" 2-bit counters for predicting conditional branches.
// this->banks: "n", the number of BTB banks, which the BPU had set equal to the maximum-length sequential fetch bundle.
// this->cond_branch_per_cycle: "m", the maximum number of conditional branches allowed in a fetch bundle.
//
// Output:
// The predicted sequential fetch bundle:
// 1. Its length (fetch_bundle_length).
// 2. BTB information (hit, type, target) at each slot within the fetch bundle (btb_fetch_bundle[]).
// 3. Its next pc, if it can be provided by the BTB (next_pc, an output of this function).
//
void btb_t::lookup(uint64_t pc, uint64_t cb_predictions, uint64_t &fetch_bundle_length, btb_output_t btb_fetch_bundle[], uint64_t &next_pc) {
   uint64_t btb_bank;
   uint64_t btb_pc;
   uint64_t set;
   uint64_t way;
   bool taken;
   uint64_t num_cond_branch = 0;
   bool terminated = false;

   for (uint64_t pos = 0; pos < banks; pos++) {	// "pos" is position of the instruction within the maximum-length sequential fetch bundle.
      convert(pc, pos, btb_bank, btb_pc);	// convert {pc, pos} to {btb_bank, btb_pc}

      // Search for the instruction in its bank.
      if (search(btb_bank, btb_pc, set, way)) {
         // BTB hit.  The BTB coordinates of this branch are {btb_bank, set, way}.
         btb_fetch_bundle[pos].hit = true;
	 btb_fetch_bundle[pos].branch_type = btb[btb_bank][set][way].branch_type;
	 btb_fetch_bundle[pos].target = btb[btb_bank][set][way].target;

         // Update LRU.
	 update_lru(btb_bank, set, way);

	 // End the fetch bundle at any taken branch or at the maximum number of conditional branches.
         if (btb_fetch_bundle[pos].branch_type == BTB_BRANCH) {
	    // The low two bits of cb_predictions correspond to the next two-bit counter to examine (because we shift it right, subsequently).
	    // From this two-bit counter, set the taken flag, accordingly.
	    taken = ((cb_predictions & 3) >= 2);

	    // Shift out the used-up 2-bit counter, to set up prediction of the next conditional branch.
	    cb_predictions = (cb_predictions >> 2);

	    // Increment the number of conditional branches in the fetch bundle.
	    num_cond_branch++;

	    if (taken || (num_cond_branch == cond_branch_per_cycle)) {
	       terminated = true;
	       fetch_bundle_length = (pos + 1);
	       next_pc = (taken ? btb_fetch_bundle[pos].target : (pc + (fetch_bundle_length << 2)));
	       break;
	    }
	 }
         else {  // All other branch types are unconditionally taken.
	    terminated = true;
	    fetch_bundle_length = (pos + 1);
	    next_pc = btb_fetch_bundle[pos].target;   // Note: valid for jump direct, call direct; invalid for jump indirect, call indirect, return.
	    break;
         }
      }
      else {
	 // BTB miss.
         btb_fetch_bundle[pos].hit = false;
      }
   }

   if (!terminated) {
      fetch_bundle_length = banks;
      next_pc = (pc + (fetch_bundle_length << 2));
   }

   assert(fetch_bundle_length <= banks);
}


//
// pc: The start pc of the fetch bundle.
// btb_miss_bit: The position of the branch in the fetch bundle, that had missed in the BTB just prior.
//
// Role of this function: Add the branch to the BTB at the correct coordinates {btb_bank, set, way}.
//
void btb_t::update(uint64_t pc, uint64_t btb_miss_bit, uint64_t btb_miss_target, insn_t insn) {
   uint64_t btb_bank;
   uint64_t btb_pc;
   uint64_t set;
   uint64_t way;

   convert(pc, btb_miss_bit, btb_bank, btb_pc);	// convert {pc, pos (btb_miss_bit)} to {btb_bank, btb_pc}
 
   // Search for the instruction in its bank.
   // It should miss, in which case the BTB entry to replace is at coordinates {btb_bank, set, way}
   assert(search(btb_bank, btb_pc, set, way) == false);

   // The entry's metadata:
   btb[btb_bank][set][way].valid = true;
   btb[btb_bank][set][way].tag = (btb_pc >> log2sets);
   update_lru(btb_bank, set, way); // Update LRU.

   // The entry's payload:
   btb[btb_bank][set][way].branch_type = decode(insn);
   btb[btb_bank][set][way].target = btb_miss_target;
}


////////////////////////////////////
// Private utility functions.
////////////////////////////////////

// Convert {pc, pos} to {btb_bank, btb_pc}, where:
// pc: start PC of a fetch bundle
// pos: position of the instruction within the fetch bundle
// btb_bank: which bank to search for this instruction.
// btb_pc: the pc of the instruction, shifted right to eliminate the low 2 bits (00) and the bank select bits after it.
void btb_t::convert(uint64_t pc, uint64_t pos, uint64_t &btb_bank, uint64_t &btb_pc) {
   // Calculate the btb bank that should be referenced.
   //
   // pc: The start pc of the fetch bundle. Shift "pc" right by 2 bits to get the instr.-level PC from the byte-level PC.   "(pc >> 2)"
   // pos: This is the instr. slot of interest in the fetch bundle.
   //      If 0, instr. is at (pc>>2)+0; if 1, instr. is at (pc>>2)+1; if 2, instr. is at (pc>>2)+2; etc.          "(pc >> 2) + pos"
   // banks: We asserted that this must be a power-of-2.
   //        Thus, BTB bank selection can be done by taking the instr. pc, "(pc >> 2) + pos", and masking it with "(banks - 1)".
   btb_bank = (((pc >> 2) + pos) & (banks - 1));

   // Discard the low two bits and the bank selection bits that follow it, from the instruction's PC.
   // The bank selection bits are implied by which bank is referenced.
   btb_pc = (((pc >> 2) + pos) >> log2banks);
}

// This function searches for the specified branch, "btb_pc", in the specified bank, "btb_bank".
// It returns true if found (hit) and false if not found (miss).
// It outputs the "set" and "way" of either (a) the branch's entry (hit) or (b) the LRU entry (which can be used by the caller for replacement).
bool btb_t::search(uint64_t btb_bank, uint64_t btb_pc, uint64_t &set, uint64_t &way) {
   // Break up btb_pc into index and tag.
   uint64_t index = (btb_pc & (sets - 1));
   uint64_t tag = (btb_pc >> log2sets);

   // Search the indexed set.
   bool hit = false;
   uint64_t hit_way = assoc; // out-of-bounds
   uint64_t lru_way = assoc; // out-of-bounds
   for (uint64_t i = 0; i < assoc; i++) {
      if (btb[btb_bank][index][i].valid && (btb[btb_bank][index][i].tag == tag)) {
         hit = true;
	 hit_way = i;
	 break;
      }
      else if (btb[btb_bank][index][i].lru == (assoc - 1)) {
         lru_way = i;
      }
   }

   // Outputs.
   set = index;
   way = (hit ? hit_way : lru_way);
   assert(way < assoc);
   return(hit);
}


void btb_t::update_lru(uint64_t btb_bank, uint64_t set, uint64_t way) {
   // Make "way" most-recently-used.
   for (uint64_t i = 0; i < assoc; i++) {
      if (btb[btb_bank][set][i].lru < btb[btb_bank][set][way].lru)
         btb[btb_bank][set][i].lru++;
   }
   btb[btb_bank][set][way].lru = 0;
}


btb_branch_type_e btb_t::decode(insn_t insn) {
   btb_branch_type_e branch_type;
   switch (insn.opcode()) {
      case OP_BRANCH:
         branch_type = BTB_BRANCH;
         break;

      case OP_JAL:
         // According to the ABI, a JAL that saves its return address into X1 is a call instruction.
         branch_type = ((insn.rd() == 1) ?  BTB_CALL_DIRECT : BTB_JUMP_DIRECT);
         break;

      case OP_JALR:
         if ((insn.rd() == 0) && (insn.rs1() == 1)) {
	    // According to the ABI, a JALR that discards its return address (writes X0) and jumps to the link register (X1) is a return instruction.
	    branch_type = BTB_RETURN;
         }
	 else if (insn.rd() == 1) {
	    // According to the ABI, a JALR that saves its return address into X1 is a call instruction.
	    branch_type = BTB_CALL_INDIRECT;
         }
	 else {
            branch_type = BTB_JUMP_INDIRECT;
	 }
         break;

      default:
         assert(0);
         break;
   }
   return(branch_type);
}




#if 0
// RISCV ISA spec, Table 2.1, explains rules for inferring a call instruction.
if (is_link_reg(insn.rd()))
   ras.push(pc + 4);

bool bpu_t::is_link_reg(uint64_t x) {
   return((x == 1) || (x == 5));
}
#endif


