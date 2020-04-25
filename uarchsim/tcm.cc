#include <cinttypes>
#include <cassert>
#include <cmath>

#include "processor.h"
#include "decode.h"
#include "config.h"

#include "tcm.h"
#include <iostream>
#include <stdio.h>
#include "parameters.h"
using namespace std;

tcm_t::tcm_t(uint64_t num_entries, uint64_t assoc, uint64_t num_instr_per_cycle, uint64_t cond_branch_per_cycle)
{
	this->sets = (num_entries/(num_instr_per_cycle*assoc)); 
	this->assoc = assoc;
	this->num_instr_per_cycle = num_instr_per_cycle;
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
   	bool ends_in_branch;
   	bool last_taken;
	uint64_t predictions;
	uint64_t cb_predictions_tmp = cb_predictions;
   	

        tcm_bundle_length = 0;
	predictions = 0;
   	for (int i = 0 ; i <cond_branch_per_cycle ; i++)
   	{
   		predictions = predictions | (((cb_predictions_tmp & 3) >= 2) << i);
		cb_predictions_tmp = cb_predictions_tmp >> 2;
    }
   	if (search(pc, cb_predictions, set, way))
   	{
         // TCM hit.  The TCM coordinates of this branch are {set, way}.
        tcm_hit = true;

        // Getting Fetch Bundle from TCM 
        for (int i =0; i< num_instr_per_cycle ; i++)
       	{
       		tcm_fetch_bundle[i] = tcm[set][way].tcm_fetch_bundle[i];
       	}

   	ends_in_branch = false;
   	last_taken = false;
        tcm_bundle_length = tcm[set][way].tcm_bundle_length;
        tcm_br_mask = tcm[set][way].br_mask;
	for (int i = 0; i < cond_branch_per_cycle; i++) {
		tcm_br_mask = tcm_br_mask >> 1;
		if((tcm_br_mask & 1) == 1)
			predictions = predictions >> 1;
	}
	if ((predictions & 1) == 1) {
		last_taken = true; // Check if mth branch is taken/not-taken
	}

        if((tcm[set][way].ends_in_br==1) && last_taken)
        {
        	tcm_next_pc = tcm[set][way].tcm_fetch_bundle[tcm[set][way].tcm_bundle_length-1].target; // Taken Target of Last Branch
        }
        else if((tcm[set][way].ends_in_br==0) || !last_taken)
        {
        	tcm_next_pc = tcm[set][way].fall_thru_pc; // Fallthrough-pc 
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
void tcm_t::line_fill_buffer(uint64_t pc, uint64_t cb_predictions, uint64_t fetch_bundle_length, btb_output_t btb_fetch_bundle[], uint64_t next_pc){
	if(valid_line_fill==0){
		pc_line_fill = pc;
		cb_predictions_line_fill = cb_predictions;
		valid_line_fill = 1;
		cond_branch_line_fill = 0;
		br_cntr_line_fill = 0;
		br_mask_line_fill = 0;
		line_fill_buffer_entry.ends_in_br = 0;
		line_fill_buffer_entry.valid = 1;
		line_fill_buffer_entry.tcm_bundle_length = 0;
	}
	uint64_t pos = line_fill_buffer_entry.tcm_bundle_length;
	uint64_t bundle_len = 0;
	uint64_t br_fall_thru = pc;
	uint64_t br_next_pc = pc;
	uint64_t last_br = 0;
	uint64_t indirect = 0;
   	for (uint64_t i = 0; i < fetch_bundle_length; i++) {
		if (((i + pos) < num_instr_per_cycle) && (br_mask_line_fill < ((1 << cond_branch_per_cycle)-1))){
			bundle_len++;
			last_br = 0;
			br_next_pc = INCREMENT_PC(br_next_pc);
			if ((btb_fetch_bundle[i].branch_type == BTB_BRANCH) && btb_fetch_bundle[i].hit){
				br_fall_thru = br_next_pc;
				last_br = 1;
				cond_branch_line_fill = ((cond_branch_line_fill << 1) | ((cb_predictions & 3) >= 2));
	    			br_mask_line_fill = ((br_mask_line_fill << 1) | 1);
				if ((cb_predictions & 3) >= 2)
					br_next_pc = btb_fetch_bundle[i].target;
				cb_predictions = cb_predictions >> 2;
				br_cntr_line_fill++;
			}
			else if ((btb_fetch_bundle[i].branch_type == BTB_JUMP_DIRECT) && btb_fetch_bundle[i].hit){ //Jump directs should not contribute to conditional branches
				br_fall_thru = btb_fetch_bundle[i].target;
				br_next_pc = btb_fetch_bundle[i].target;
			}
			line_fill_buffer_entry.tcm_fetch_bundle[i+pos] = btb_fetch_bundle[i];
			if (((btb_fetch_bundle[i].branch_type == BTB_CALL_DIRECT) || 
			     (btb_fetch_bundle[i].branch_type == BTB_JUMP_INDIRECT) ||
			     (btb_fetch_bundle[i].branch_type == BTB_CALL_INDIRECT) ||
			     (btb_fetch_bundle[i].branch_type == BTB_RETURN))
			     && btb_fetch_bundle[i].hit) {
				indirect = 1;
				line_fill_full = 1;
				break;
			}
		}
	}
	if ((indirect == 1) || ((btb_fetch_bundle[bundle_len-1].branch_type == BTB_JUMP_DIRECT) && btb_fetch_bundle[bundle_len-1].hit)) {
		line_fill_buffer_entry.fall_thru_pc = btb_fetch_bundle[bundle_len-1].target; //equal to pc incremented by fetch bundle width BUG_FIX
	}
	else {
		if (last_br == 1)
			line_fill_buffer_entry.fall_thru_pc = br_fall_thru; //equal to pc incremented by fetch bundle width BUG_FIX
		else
			line_fill_buffer_entry.fall_thru_pc = br_next_pc; //equal to pc incremented by fetch bundle width BUG_FIX
	}
	indirect = 0;
	line_fill_buffer_entry.tcm_bundle_length = line_fill_buffer_entry.tcm_bundle_length + bundle_len;
	assert(line_fill_buffer_entry.tcm_bundle_length <= num_instr_per_cycle);
	assert(br_mask_line_fill <= ((1 << cond_branch_per_cycle)-1));
	if((line_fill_buffer_entry.tcm_bundle_length == num_instr_per_cycle) || (br_mask_line_fill == ((1 << cond_branch_per_cycle)-1))) {
		line_fill_full = 1;
	}
	if(line_fill_full == 1){
		if (FILL_ON_TAKEN_BRANCH == 1) {
			if (cond_branch_line_fill > 0) {
				if (last_br == 1){
					line_fill_buffer_entry.ends_in_br = 1;
					br_mask_line_fill = br_mask_line_fill >> 1;
					cond_branch_line_fill = cond_branch_line_fill >> 1;
					br_cntr_line_fill--;
				}
				if (br_cntr_line_fill < cond_branch_per_cycle) {
					for (uint64_t i = 0; i < (cond_branch_per_cycle - br_cntr_line_fill); i++){
						br_mask_line_fill = (br_mask_line_fill << 1);
						cond_branch_line_fill = (cond_branch_line_fill << 1);
					}
				}
				commit_line_fill();
			}
			else {
				clear_line_fill();
			}
		}
		else {
			if (last_br == 1){
				line_fill_buffer_entry.ends_in_br = 1;
				br_mask_line_fill = br_mask_line_fill >> 1;
				cond_branch_line_fill = cond_branch_line_fill >> 1;
				br_cntr_line_fill--;
			}
			if (br_cntr_line_fill < cond_branch_per_cycle) {
				for (uint64_t i = 0; i < (cond_branch_per_cycle - br_cntr_line_fill); i++){
					br_mask_line_fill = (br_mask_line_fill << 1);
					cond_branch_line_fill = (cond_branch_line_fill << 1);
				}
			}
			commit_line_fill();
		}
	}
}


void tcm_t::commit_line_fill() {
	bool hit;
	uint64_t commit_set;
	uint64_t commit_way;
	if (line_fill_full == 1){
		valid_line_fill = 0;
		line_fill_full = 0;
		hit = search(pc_line_fill, cb_predictions_line_fill, commit_set, commit_way);
		update_lru(commit_set, commit_way);
		tcm[commit_set][commit_way].valid = line_fill_buffer_entry.valid;
		tcm[commit_set][commit_way].tag = (pc_line_fill >> log2sets);
		tcm[commit_set][commit_way].br_flags = cond_branch_line_fill;
		tcm[commit_set][commit_way].br_mask = br_mask_line_fill;
		tcm[commit_set][commit_way].tcm_bundle_length = line_fill_buffer_entry.tcm_bundle_length;
		tcm[commit_set][commit_way].ends_in_br = line_fill_buffer_entry.ends_in_br;
   		for (uint64_t i = 0; i < num_instr_per_cycle; i++)
			tcm[commit_set][commit_way].tcm_fetch_bundle[i] = line_fill_buffer_entry.tcm_fetch_bundle[i];
		tcm[commit_set][commit_way].fall_thru_pc = line_fill_buffer_entry.fall_thru_pc; // fall through address to be used in case bundle does not en with a branch or branch is not taken BUG_FIX
		line_fill_buffer_entry.valid = 0;
	}
}





/////////////////////////
/// Private Functions////
/////////////////////////

bool tcm_t::search(uint64_t pc, uint64_t cb_predictions, uint64_t &set, uint64_t &way)
{
	uint64_t tcm_pc = pc ;  
	uint64_t index = (tcm_pc & (sets - 1)); 
   	uint64_t tag = (tcm_pc >> log2sets);
   	uint64_t predictions;

   	predictions = 0;
   	for (int i = 0 ; i <cond_branch_per_cycle ; i++)
   	{
   		predictions = predictions | ((cb_predictions & 3) >= 2);
		cb_predictions = cb_predictions >> 2;
   		predictions = predictions << 1;
    }
   		predictions = predictions >> 2;
	   
   
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
		if (tcm[index][i].valid && (tcm[index][i].tag == tag) && ((br_mask_search & predictions) == (br_mask_search & br_flags_search))) 
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

void tcm_t::clear_line_fill() {
	valid_line_fill = 0;
}
