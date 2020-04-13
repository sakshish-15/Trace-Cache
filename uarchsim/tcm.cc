#include <cinttypes>
#include <cassert>
#include <cmath>

#include "processor.h"
#include "decode.h"
#include "config.h"

#include "tcm.h"

tcm_t::tcm_t(uint64_t num_entries, uint64_t assoc, uint64_t cond_branch_per_cycle)
{
	this->sets = (num_entries/(MAX_TCM_BUNDLE*assoc)); 
	this->assoc = assoc;
	this->cond_branch_per_cycle = cond_branch_per_cycle;

	assert(IsPow2(sets));

	log2sets = (uint64_t) log2((double)sets);

	// Allocate the 2D array.
	tcm = new tcm_entry_t *[sets];
	
	for (uint64_t s = 0; s < sets; s++) 
	{
		tcm[s] = new tcm_entry_t[assoc];
	 	for (uint64_t way = 0; way < assoc; way++)
	 	{
	    	tcm[s][way].valid = false;
	    	tcm[s][way].lru = way;
	 	}  
	}
}


tcm_t::~tcm_t()
{

}

//Inputs:
// 1. pc: The start pc of the fetch bundle.
// 2. cb_predictions: A uint64_t packed with "m" 2-bit counters for predicting conditional branches.
// this->cond_branch_per_cycle: "m", the maximum number of conditional branches allowed in a fetch bundle.

// Output:
// The predicted sequential fetch bundle:
// 1. True if TCM hit else returns False
// 2. Its length (fetch_bundle_length).
// 3. BTB information (hit, type, target) at each slot within the fetch bundle (btb_fetch_bundle[]).
// 4. Its next pc, if it can be provided by the BTB (next_pc, an output of this function).
//
bool tcm_t::lookup(uint64_t pc, uint64_t cb_predictions, uint64_t &tcm_bundle_length, btb_output_t tcm_fetch_bundle[], uint64_t &tcm_next_pc)
{
	uint64_t set;
   	uint64_t way;
   	uint64_t num_cond_branch = 0;
   	uint64_t tcm_br_mask;
   	uint64_t tcm_hit;
   	uint64_t ends_in_branch;
   	uint64_t last_taken;
	uint64_t predictions;
   	

   	for (int i = 0 ; i <cond_branch_per_cycle ; i++)
   	{
   		predictions = predictions | ((cb_predictions & 3) >= 2);
   		predictions = predictions << 1;
    }
   
   	if (search(pc, cb_predictions, set, way))
   	{
         // TCM hit.  The TCM coordinates of this branch are {set, way}.
        tcm_hit = true;

        // Getting Fetch Bundle from TCM 
        for (int i =0; i< MAX_TCM_BUNDLE ; i++)
       	{
       		tcm_fetch_bundle[i] = tcm[set][way].tcm_fetch_bundle[i];
       	}

        tcm_bundle_length = tcm[set][way].tcm_bundle_length;
        ends_in_branch = tcm[set][way].br_mask && 1; // branch mask is m bit and last bit tells if trace ends in branch
        last_taken = predictions && 1; // Check if mth branch is taken/not-taken

        //Fall through (!ends_in_branch || last_not_taken)
        //Trace Target (ends_in_branch && last_taken)

        if(ends_in_branch && last_taken)
        {
        	tcm_next_pc = tcm[set][way].tcm_fetch_bundle[tcm[set][way].tcm_bundle_length-1].target; // Taken Target of Last Branch
        }
        else if(!ends_in_branch || !last_taken)
        {
        	tcm_next_pc = pc + (tcm[set][way].tcm_bundle_length << 2); // Fallthrough-pc 
        }

         // Update LRU.
	 	update_lru(set, way);

	}

    else 
    {
	 // TCM miss.
         tcm_hit = false;
    }

   assert(tcm_bundle_length <= num_instr_per_cycle);
   return(tcm_hit);

}

// Called after btb is hit and fetch bundle is formed
void tcm_t::line_fill_buffer(uint64_t pc, uint64_t cb_predictions, uint64_t fetch_bundle_length, btb_output_t btb_fetch_bundle[], uint64_t next_pc)
{
	assert(pc == line_fill_buffer_entry.next_pc);
	checkpoint_line_fill();
	if(valid_line_fill==0)
	{
		pc_line_fill = pc;
		cb_predictions_line_fill = cb_predictions;
		valid_line_fill = 1;
		cond_branch_line_fill = 0;
		br_cntr_line_fill = 0;
		br_mask_line_fill = 0;
		line_fill_buffer_entry.tcm_bundle_length = 0;
	}
	uint64_t pos = line_fill_buffer_entry.tcm_bundle_length;
	uint64_t bundle_len = 0;
	if ((line_fill_buffer_entry.tcm_bundle_length + fetch_bundle_length) > MAX_TCM_BUNDLE)
		line_fill_buffer_entry.tcm_bundle_length = MAX_TCM_BUNDLE;
	else
		line_fill_buffer_entry.tcm_bundle_length = line_fill_buffer_entry.tcm_bundle_length + fetch_bundle_length;
   	for (uint64_t i = 0; i < fetch_bundle_length; i++) 
   	{
		if (((i + pos) < MAX_TCM_BUNDLE) && (cond_branch_line_fill != ((1 << cond_branch_per_cycle)-1)))
		{
			bundle_len++;
			if (btb_fetch_bundle[i].branch_type == BTB_BRANCH)
			{
				cond_branch_line_fill = ((cond_branch_line_fill << 1) | 1);
	    			br_mask_line_fill = ((br_mask_line_fill << 1) | ((cb_predictions & 3) >= 2));
				cb_predictions = cb_predictions >> 2;
				br_cntr_line_fill++;
			}
			else if (btb_fetch_bundle[i].branch_type == BTB_JUMP_DIRECT)
			{
				cond_branch_line_fill = ((cond_branch_line_fill << 1) | 1);
	    			br_mask_line_fill = ((br_mask_line_fill << 1) | 1);
				cb_predictions = cb_predictions >> 2;
				br_cntr_line_fill++;
			}
			line_fill_buffer_entry.tcm_fetch_bundle[i+pos] = btb_fetch_bundle[i];
		}
		else 
		{
			line_fill_full = 1;
			break;
		}
	}
	line_fill_buffer_entry.fall_thru_pc = next_pc;
	line_fill_buffer_entry.next_pc = next_pc;
	line_fill_buffer_entry.tcm_bundle_length = line_fill_buffer_entry.tcm_bundle_length + bundle_len;
	assert(line_fill_buffer_entry.tcm_bundle_length <= MAX_TCM_BUNDLE);
	assert(cond_branch_line_fill <= ((1 << cond_branch_per_cycle)-1));
}






void tcm_t::rollback_line_fill()
{
	valid_line_fill = valid;
	cond_branch_line_fill = cond_branch;
	br_mask_line_fill = br_mask;
	br_cntr_line_fill = br_cntr;
	line_fill_buffer_entry.tcm_bundle_length = checkpoint_line.tcm_bundle_length;
   	for (uint64_t i = 0; i < MAX_TCM_BUNDLE; i++)
		line_fill_buffer_entry.tcm_fetch_bundle[i] = checkpoint_line.tcm_fetch_bundle[i];
	line_fill_buffer_entry.next_pc = checkpoint_line.next_pc;
	line_fill_full = 0;
}



void tcm_t::commit_line_fill() 
{
	bool hit;
	uint64_t commit_set;
	uint64_t commit_way;
	if (line_fill_full == 1)
	{
		valid_line_fill = 0;
		line_fill_full = 0;
		hit = search(pc_line_fill, cb_predictions_line_fill, commit_set, commit_way);
		update_lru(commit_set, commit_way);
		tcm[commit_set][commit_way].valid = valid;
		tcm[commit_set][commit_way].tag = (pc_line_fill >> log2sets);
		if (br_cntr_line_fill < cond_branch_per_cycle) 
		{
			for (uint64_t i = 0; i < (cond_branch_per_cycle - br_cntr_line_fill); i++)
			{
				cond_branch_line_fill = (cond_branch_line_fill << 1);
				br_mask_line_fill = (br_mask_line_fill << 1);
			}
		}
		tcm[commit_set][commit_way].br_flags = cond_branch_line_fill;
		tcm[commit_set][commit_way].br_mask = br_mask_line_fill;
		tcm[commit_set][commit_way].tcm_bundle_length = line_fill_buffer_entry.tcm_bundle_length;
   		for (i = 0; i < MAX_TCM_BUNDLE; i++)
			tcm[commit_set][commit_way].tcm_fetch_bundle[i] = line_fill_buffer_entry.tcm_fetch_bundle[i];
		tcm[commit_set][commit_way].next_pc = line_fill_buffer_entry.next_pc;
	}
}





/////////////////////////
/// Private Functions////
/////////////////////////

bool tcm_t::search(uint64_t pc, uint64_t cb_predictions, uint64_t &set, uint64_t &way)
{
	uint64_t tcm_pc = (pc >> 2); // Discard the lower two bits 
	uint64_t index = (tcm_pc & (sets - 1)); 
   	uint64_t tag = (tcm_pc >> log2sets);
   	uint64_t predictions;
   	

   	for (int i = 0 ; i <cond_branch_per_cycle ; i++)
   	{
   		predictions = predictions | ((cb_predictions & 3) >= 2);
   		predictions = predictions << 1;
    }
	   
   
   // Search the indexed set.
   bool hit = false;
   uint64_t hit_way = assoc; // out-of-bounds
   uint64_t lru_way = assoc; // out-of-bounds
   uint64_t br_mask_search;
   uint64_t br_flags_search;
   
   for (uint64_t i = 0; i < assoc; i++) 
   {
   		br_mask_search = (tcm[index][i].br_mask >> 1);
   		br_flags_search = (tcm[index][i].br_flags >> 1);

   		// Trace Cache Hit if Index is Valid, Tag match and m predictions from predictor matches with the predictions saved in TCM
		if (tcm[index][i].valid && (tcm[index][i].tag == tag) && ((br_mask_search && predictions) == (br_mask_search && br_flags_search))) 
		{
			hit = true;
			hit_way = i;
			break;
		}

    	else if (tcm[index][i].lru == (assoc - 1)) 
      	{
        	lru_way = i;
      	}
   }

   // Outputs.
   set = index;
   way = (hit ? hit_way : lru_way);
   assert(way < assoc);
   return(hit);
}


void tcm_t::update_lru(uint64_t set, uint64_t way)
{
	// Make "way" most-recently-used.
	for (uint64_t i = 0; i < assoc; i++)
	{
		if (tcm[set][i].lru < tcm[set][way].lru)
		tcm[set][i].lru++;
	}
	tcm[set][way].lru = 0;
}

void tcm_t::checkpoint_line_fill()
{
	valid = valid_line_fill;
	cond_branch = cond_branch_line_fill;
	br_mask = br_mask_line_fill;
	br_cntr = br_cntr_line_fill;
	checkpoint_line.tcm_bundle_length = line_fill_buffer_entry.tcm_bundle_length;
   	for (uint64_t i = 0; i < MAX_TCM_BUNDLE; i++)
		checkpoint_line.tcm_fetch_bundle[i] = line_fill_buffer_entry.tcm_fetch_bundle[i];
	checkpoint_line.next_pc = line_fill_buffer_entry.next_pc;
}



btb_branch_type_e tcm_t::decode(insn_t insn) 
{
   btb_branch_type_e branch_type;
   switch (insn.opcode()) 
   {
      case OP_BRANCH:
         branch_type = BTB_BRANCH;
         break;

      case OP_JAL:
         // According to the ABI, a JAL that saves its return address into X1 is a call instruction.
         branch_type = ((insn.rd() == 1) ?  BTB_CALL_DIRECT : BTB_JUMP_DIRECT);
         break;

      case OP_JALR:
        if ((insn.rd() == 0) && (insn.rs1() == 1)) 
        {
	    	// According to the ABI, a JALR that discards its return address (writes X0) and jumps to the link register (X1) is a return instruction.
	    	branch_type = BTB_RETURN;
        }
	 	else if (insn.rd() == 1) 
	 	{
	    // According to the ABI, a JALR that saves its return address into X1 is a call instruction.
	    	branch_type = BTB_CALL_INDIRECT;
        }
	 	else 
	 	{
            branch_type = BTB_JUMP_INDIRECT;
	 	}
        break;

      default:
         assert(0);
         break;
   }
   return(branch_type);
}








