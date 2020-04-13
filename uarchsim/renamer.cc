// Author : Sakshi
// RENAMER CLASS
// Version: 3
// Last Modified 12 Feb 2020

#include <inttypes.h>
#include "assert.h"
#include <iostream>
#include <stdio.h>
#include "renamer.h"
using namespace std;

	////////////////////////////////////////
	// Public functions.
	////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// This is the constructor function.
	// When a renamer object is instantiated, the caller indicates:
	// 1. The number of logical registers (e.g., 32).
	// 2. The number of physical registers (e.g., 128).
	// 3. The maximum number of unresolved branches.
	//    Requirement: 1 <= n_branches <= 64.
	//
	// Tips:
	//
	// Assert the number of physical registers > number logical registers.
	// Assert 1 <= n_branches <= 64.
	// Then, allocate space for the primary data structures.
	// Then, initialize the data structures based on the knowledge
	// that the pipeline is intially empty (no in-flight instructions yet).
	/////////////////////////////////////////////////////////////////////
	renamer::renamer(uint64_t n_log_regs,uint64_t n_phys_regs,uint64_t n_branches)
	{
	//	cout << "Constructor called" << endl;
		
		assert(n_phys_regs > n_log_regs);
		assert( (n_branches <= 64) && (n_branches >=1));
		
		//Size of Active and Free list
		size_free_list = n_phys_regs - n_log_regs;
		size_active_list = n_phys_regs - n_log_regs;

		this->n_phys_regs = n_phys_regs;
		this->n_log_regs = n_log_regs;
		this->n_branches = n_branches;
		this->head_free_list = 0;
		this->head_active_list = 0;
		this->tail_free_list = 0;
		this->tail_active_list = 0;
		this->size_free_entry_free_list = 0;
		this->size_free_entry_active_list = size_active_list;


		//Allocating Memory for all 8 structures
		Active_list = new active_list_struct [size_active_list];
		Free_list = new uint64_t [size_free_list];

		RMT = new uint64_t[n_log_regs];
		AMT = new uint64_t[n_log_regs];
		
		Physical_Reg_File = new uint64_t[n_phys_regs];
		PRF_ready_array = new bool[n_phys_regs];

		Branch_Checkpoint = new Branch_Checkpoint_struct[n_branches];

		for (int i =0 ; i < n_branches ; i++)
		{
			Branch_Checkpoint[i].Shadow_Map_Table = new uint64_t [n_log_regs];	
		}

		for (int i =0 ; i < n_branches ; i++)
		{
			Branch_Checkpoint[i].Checkpointed_GBM = 0;
			Branch_Checkpoint[i].Checkpointed_Free_list_head_index = 0;
			for (int j = 0 ; j < n_log_regs ; j++)
			{
				Branch_Checkpoint[i].Shadow_Map_Table[j] = 0;	
			}
		}	
		

		// Initializng the Structures
		GBM = 0;
		for (int i =0 ; i < n_log_regs ; i++)
		{
			RMT[i] = i;
			AMT[i] = i;
			PRF_ready_array[i] = true;
		}
		for(int i =n_log_regs; i< n_phys_regs;i++)
		{
			PRF_ready_array[i] = 'x';
		}


		// Free List Free regs of PRF (except the ones in AMT)
		for (int i = 0 ; i < size_free_list; i++)
		{
			Free_list[i] = i + n_log_regs;
		}


		// Active list initially empty
		for (int i=0; i < size_active_list ; i++)
		{	
			Active_list[i].dest_flag = false;
			Active_list[i].log_reg_num_dest = 'x';
			Active_list[i].phys_reg_num_dest = 'x';
			Active_list[i].completed_bit = false;
			Active_list[i].exception_bit = false;
			Active_list[i].load_violation_bit = false;
			Active_list[i].branch_misprediction_bit = false;
			Active_list[i].value_misprediction_bit = false;
			Active_list[i].load_flag = false;
			Active_list[i].store_flag = false;
			Active_list[i].branch_flag = false;
			Active_list[i].amo_flag = false;
			Active_list[i].csr_flag = false;
			Active_list[i].PC = 'x';
		}
	}

	/////////////////////////////////////////////////////////////////////
	// This is the destructor, used to clean up memory space and
	// other things when simulation is done.
	// I typically don't use a destructor; you have the option to keep
	// this function empty.
	/////////////////////////////////////////////////////////////////////
	//~renamer();
	/////////////////////////////////////////////////////////////////////
	// The Rename Stage must stall if there aren't enough free physical
	// registers available for renaming all logical destination registers
	// in the current rename bundle.
	//
	// Inputs:
	// 1. bundle_dst: number of logical destination registers in
	//    current rename bundle
	//
	// Return value:
	// Return "true" (stall) if there aren't enough free physical
	// registers to allocate to all of the logical destination registers
	// in the current rename bundle.
	/////////////////////////////////////////////////////////////////////

	bool renamer::stall_reg(uint64_t bundle_dst)
	{
	
		if(size_free_list - size_free_entry_free_list < bundle_dst)
		return true;
		else
		return false;
	}

	/////////////////////////////////////////////////////////////////////
	// The Rename Stage must stall if there aren't enough free
	// checkpoints for all branches in the current rename bundle.
	//
	// Inputs:
	// 1. bundle_branch: number of branches in current rename bundle
	//
	// Return value:
	// Return "true" (stall) if there aren't enough free checkpoints
	// for all branches in the current rename bundle.
	/////////////////////////////////////////////////////////////////////
	bool renamer:: stall_branch(uint64_t bundle_branch)
	{
		//////printf("Stall Rename if not enough free Checlpoints\n");
		// Check if GBM is Empty
		int no_of_zeroes = 0;
		for (int i =0 ; i < n_branches;i++)
		{
			int p = i;
			uint64_t mask = 1 << p;
			if((GBM & mask) == 0)
			{
				no_of_zeroes++;
			}
		}

			
		if(no_of_zeroes < bundle_branch)
		{
			return true;
		}

		else
		{
			return false;
		}
	}

	/////////////////////////////////////////////////////////////////////
	// This function is used to get the branch mask for an instruction.
	/////////////////////////////////////////////////////////////////////
	uint64_t renamer::get_branch_mask()
	{
		//printf("Return Branch Mask\n");
		return GBM;
	}


	/////////////////////////////////////////////////////////////////////
	// This function is used to rename a single source register.
	//
	// Inputs:
	// 1. log_reg: the logical register to rename
	//
	// Return value: physical register name
	/////////////////////////////////////////////////////////////////////
	uint64_t renamer::rename_rsrc(uint64_t log_reg)
	{
		//printf("Rename src\t");
		uint64_t physical_reg_name;
		physical_reg_name = RMT[log_reg];
		return physical_reg_name;
	}
	/////////////////////////////////////////////////////////////////////
	// This function is used to rename a single destination register.
	//
	// Inputs:
	// 1. log_reg: the logical register to rename
	//
	// Return value: physical register name
	/////////////////////////////////////////////////////////////////////
	uint64_t renamer::rename_rdst(uint64_t log_reg)
	{
		//printf("Rename dest\t");
		uint64_t physical_reg_name;

  	
		assert (size_free_list  - size_free_entry_free_list > 0);
		assert (stall_reg(1) == false);
		
		physical_reg_name = Free_list[head_free_list]; // Pop Free PR from FRee list
		RMT[log_reg] = physical_reg_name;	 // Save Mapping in RMT

		//Updating free entries in Free List

		if(head_free_list == size_free_list -1 )
		{
			head_free_list = 0;
			size_free_entry_free_list ++;
		}
		else
		{
			head_free_list++;
			size_free_entry_free_list ++;
		}


		return physical_reg_name;
	}

	/////////////////////////////////////////////////////////////////////
	// This function creates a new branch checkpoint.
	//
	// Inputs: none.
	//
	// Output:
	// 1. The function returns the branch's ID. When the branch resolves,
	//    its ID is passed back to the renamer via "resolve()" below.
	//
	// Tips:
	//
	// Allocating resources for the branch (a GBM bit and a checkpoint):
	// * Find a free bit -- i.e., a '0' bit -- in the GBM. Assert that
	//   a free bit exists: it is the user's responsibility to avoid
	//   a structural hazard by calling stall_branch() in advance.
	// * Set the bit to '1' since it is now in use by the new branch.
	// * The position of this bit in the GBM is the branch's ID.
	// * Use the branch checkpoint that corresponds to this bit.
	// 
	// The branch checkpoint should contain the following:
	// 1. Shadow Map Table (checkpointed Rename Map Table)
	// 2. checkpointed Free List head index
	// 3. checkpointed GBM
	/////////////////////////////////////////////////////////////////////
	uint64_t renamer::checkpoint()
	{	
		uint64_t Branch_id;
		//printf("Checkpoint\n");
		int no_of_zeroes = 0;
		uint64_t mask;

		for (uint64_t i =0 ; i < n_branches;i++)
		{
			mask = 1 << i;
			if((GBM & mask) == 0)
			{
				no_of_zeroes++;
			}
		}
		assert(no_of_zeroes > 0);
		assert (stall_branch(1) == false);
		
			for (uint64_t i =0 ; i < n_branches;i++)
			{
				//uint64_t p = i;
				mask = 1 << i;
				if((GBM & mask) == 0)
				{
					Branch_id = i;
					GBM = GBM | mask;
					//printf("\n Checkpoint Brach ID Assigned %d\t",Branch_id);	
			
					for(int j=0 ; j< n_log_regs; j++)
					{
						Branch_Checkpoint[Branch_id].Shadow_Map_Table[j] = RMT[j];
					}

					Branch_Checkpoint[Branch_id].Checkpointed_GBM = GBM;
					Branch_Checkpoint[Branch_id].Checkpointed_Free_list_head_index = head_free_list;
					break;
				}
			}
		
			return Branch_id;
	}
			
			
			//return Branch_id;
			
		//}
		
	

	//////////////////////////////////////////
	// Functions related to Dispatch Stage. //
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// The Dispatch Stage must stall if there are not enough free
	// entries in the Active List for all instructions in the current
	// dispatch bundle.
	//
	// Inputs:
	// 1. bundle_inst: number of instructions in current dispatch bundle
	//
	// Return value:
	// Return "true" (stall) if the Active List does not have enough
	// space for all instructions in the dispatch bundle.
	/////////////////////////////////////////////////////////////////////
	bool renamer::stall_dispatch(uint64_t bundle_inst)
	{
		//printf("Stall Dispatch\n");
	
		if (size_free_entry_active_list < bundle_inst)
		{
			return true;
		}
	
		else
		{
			return false;
		}
		
	}

	/////////////////////////////////////////////////////////////////////
	// This function dispatches a single instruction into the Active
	// List.
	//
	// Inputs:
	// 1. dest_valid: If 'true', the instr. has a destination register,
	//    otherwise it does not. If it does not, then the log_reg and
	//    phys_reg inputs should be ignored.
	// 2. log_reg: Logical register number of the instruction's
	//    destination.
	// 3. phys_reg: Physical register number of the instruction's
	//    destination.
	// 4. load: If 'true', the instr. is a load, otherwise it isn't.
	// 5. store: If 'true', the instr. is a store, otherwise it isn't.
	// 6. branch: If 'true', the instr. is a branch, otherwise it isn't.
	// 7. amo: If 'true', this is an atomic memory operation.
	// 8. csr: If 'true', this is a system instruction.
	// 9. PC: Program counter of the instruction.
	//
	// Return value:
	// Return the instruction's index in the Active List.
	//
	// Tips:
	//
	// Before dispatching the instruction into the Active List, assert
	// that the Active List isn't full: it is the user's responsibility
	// to avoid a structural hazard by calling stall_dispatch()
	// in advance.
	/////////////////////////////////////////////////////////////////////
	uint64_t renamer::dispatch_inst(bool dest_valid,
	                       uint64_t log_reg,
	                       uint64_t phys_reg,
	                       bool load,
	                       bool store,
	                       bool branch,
	                       bool amo,
	                       bool csr,
	                       uint64_t PC)
	{
		//printf("Dispatch Bundle\n");
		uint64_t AL_index;
		
		//Check if Active List isn't full
		assert (size_free_entry_active_list > 0);
		assert (stall_dispatch(1) == false);
		// Call Stall_dispatch();

		Active_list[tail_active_list].dest_flag = dest_valid;
		if(dest_valid == true)
		{
			Active_list[tail_active_list].log_reg_num_dest = log_reg;
			Active_list[tail_active_list].phys_reg_num_dest = phys_reg;
		}
		Active_list[tail_active_list].load_flag = load;
		Active_list[tail_active_list].store_flag = store;
		Active_list[tail_active_list].branch_flag = branch;
		Active_list[tail_active_list].amo_flag = amo;
		Active_list[tail_active_list].csr_flag = csr;
		Active_list[tail_active_list].PC = PC;
		Active_list[tail_active_list].exception_bit = false;
		Active_list[tail_active_list].completed_bit = false;
		Active_list[tail_active_list].load_violation_bit = false;
		Active_list[tail_active_list].branch_misprediction_bit = false;
		Active_list[tail_active_list].value_misprediction_bit = false;

		//PRF_ready_array[phys_reg] = false;

		AL_index = tail_active_list;
		
		if(tail_active_list == size_active_list -1)
		{
			tail_active_list =0;
			size_free_entry_active_list --;
		}
		else
		{
			tail_active_list++;
			size_free_entry_active_list --;
			
			
		}
		//printf("DISPATCH: Active List empty space %d and Head %d and Tail is %d and Index is %d" , size_free_entry_active_list, head_active_list,tail_active_list,AL_index);
		return AL_index;

	}


	//////////////////////////////////////////
	// Functions related to Schedule Stage. //
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Test the ready bit of the indicated physical register.
	// Returns 'true' if ready.
	/////////////////////////////////////////////////////////////////////
	bool renamer::is_ready(uint64_t phys_reg)
	{
		//printf("Check if Rdy\n");
		return PRF_ready_array[phys_reg];
	}

	/////////////////////////////////////////////////////////////////////
	// Clear the ready bit of the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	void renamer::clear_ready(uint64_t phys_reg)
	{
		////printf("Clear Rdy\n");
		PRF_ready_array[phys_reg] = false;
	}

	/////////////////////////////////////////////////////////////////////
	// Set the ready bit of the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	void renamer::set_ready(uint64_t phys_reg)
	{

		PRF_ready_array[phys_reg] = true;
	}


	//////////////////////////////////////////
	// Functions related to Reg. Read Stage.//
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Return the contents (value) of the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	uint64_t renamer::read(uint64_t phys_reg)
	{
		//printf("Read value for Reg %d\n",phys_reg);
		return Physical_Reg_File[phys_reg];
	}


	//////////////////////////////////////////
	// Functions related to Writeback Stage.//
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Write a value into the indicated physical register.
	/////////////////////////////////////////////////////////////////////
	void renamer::write(uint64_t phys_reg, uint64_t value)
	{
		////printf("Write for Reg %d\n",phys_reg);
		Physical_Reg_File[phys_reg] = value;
		//////printf("Value of %d Reg is %d\n",phys_reg,value);
	}

	/////////////////////////////////////////////////////////////////////
	// Set the completed bit of the indicated entry in the Active List.
	/////////////////////////////////////////////////////////////////////
	void renamer::set_complete(uint64_t AL_index)
	{
		//printf("Set complete\n");
// for REG %d at index %d\n", Active_list[AL_index].phys_reg_num_dest,AL_index);
		Active_list[AL_index].completed_bit = true;
	}

	/////////////////////////////////////////////////////////////////////
	// This function is for handling branch resolution.
	//
	// Inputs:
	// 1. AL_index: Index of the branch in the Active List.
	// 2. branch_ID: This uniquely identifies the branch and the
	//    checkpoint in question.  It was originally provided
	//    by the checkpoint function.
	// 3. correct: 'true' indicates the branch was correctly
	//    predicted, 'false' indicates it was mispredicted
	//    and recovery is required.
	//
	// Outputs: none.
	//
	// Tips:
	//
	// While recovery is not needed in the case of a correct branch,
	// some actions are still required with respect to the GBM and
	// all checkpointed GBMs:
	// * Remember to clear the branch's bit in the GBM.
	// * Remember to clear the branch's bit in all checkpointed GBMs.
	//
	// In the case of a misprediction:
	// * Restore the GBM from the checkpoint. Also make sure the
	//   mispredicted branch's bit is cleared in the restored GBM,
	//   since it is now resolved and its bit and checkpoint are freed.
	// * You don't have to worry about explicitly freeing the GBM bits
	//   and checkpoints of branches that are after the mispredicted
	//   branch in program order. The mere act of restoring the GBM
	//   from the checkpoint achieves this feat.
	// * Restore other state using the branch's checkpoint.
	//   In addition to the obvious state ...  *if* you maintain a
	//   freelist length variable (you may or may not), you must
	//   recompute the freelist length. It depends on your
	//   implementation how to recompute the length.
	//   (Note: you cannot checkpoint the length like you did with
	//   the head, because the tail can change in the meantime;
	//   you must recompute the length in this function.)
	// * Do NOT set the branch misprediction bit in the active list.
	//   (Doing so would cause a second, full squash when the branch
	//   reaches the head of the Active List. We don’t want or need
	//   that because we immediately recover within this function.)
	/////////////////////////////////////////////////////////////////////
	void renamer::resolve(uint64_t AL_index,
		     uint64_t branch_ID,
		     bool correct)
	{
		uint64_t mask;
		mask = 1 << branch_ID;
		//printf("\nPredited = %d\t Resolve at Index %d BranchID %d\n",correct,AL_index,branch_ID);
		//printf("\nRESOLVE before: Head  %d and Tail %d and Free entry Active List %d",head_active_list, tail_active_list,size_free_entry_active_list);
		if(correct == true) // Branch Predicted Correctly
		{
			//clear GBM bit

			GBM = (GBM & (~(mask))); 	
			/*for (int i =0 ; i < n_log_regs ; i++) // Reclaim Shadow Map Table
			{		
				//printf("Branch_ID %d", branch_ID);
				Branch_Checkpoint[branch_ID].Shadow_Map_Table[i] = 0;
			}*/

			//clear checkpointed GBMS
			for (int i =0 ; i < n_branches ; i++) 
			{
				mask = 1 << branch_ID;
				Branch_Checkpoint[i].Checkpointed_GBM = Branch_Checkpoint[i].Checkpointed_GBM & (~(mask)); 
			}

			Branch_Checkpoint[branch_ID].Checkpointed_Free_list_head_index = 0;
				
		}


		else // MisPrediction
		{
			//printf("\n\t\t MISPREDICTION ##############\n");
			GBM = Branch_Checkpoint[branch_ID].Checkpointed_GBM;
			
			mask = 1 << branch_ID;
			GBM = (GBM & (~(mask)));	
				
			// Recover RMT from Shadow Map
			for (int i =0 ; i < n_log_regs ; i++)
			{	
			//	////printf("Branch_ID %d", branch_ID);
				RMT[i] = Branch_Checkpoint[branch_ID].Shadow_Map_Table[i];
			}
			

			/*for (int i =0 ; i < n_log_regs ; i++) // Reclaim Shadow Map Table
			{
				Branch_Checkpoint[branch_ID].Shadow_Map_Table[i] = 0;
			}*/
			
			if (AL_index  < tail_active_list )
			{
				for (int i = AL_index + 1; i < tail_active_list ; i++)
				{	
					Active_list[i].dest_flag = false;
					Active_list[i].log_reg_num_dest = 'x';
					Active_list[i].phys_reg_num_dest = 'x';
					Active_list[i].completed_bit = false;
					Active_list[i].exception_bit = false;
					Active_list[i].load_violation_bit = false;
					Active_list[i].branch_misprediction_bit = false;
					Active_list[i].value_misprediction_bit = false;
					Active_list[i].load_flag = false;
					Active_list[i].store_flag = false;
					Active_list[i].branch_flag = false;
					Active_list[i].amo_flag = false;
					Active_list[i].csr_flag = false;
					Active_list[i].PC = 'x';
				}
			}

			else
			{			
				for (int i = AL_index + 1; i < size_active_list ; i++)
				{
					Active_list[i].dest_flag = false;
					Active_list[i].log_reg_num_dest = 'x';
					Active_list[i].phys_reg_num_dest = 'x';
					Active_list[i].completed_bit = false;
					Active_list[i].exception_bit = false;
					Active_list[i].load_violation_bit = false;
					Active_list[i].branch_misprediction_bit = false;
					Active_list[i].value_misprediction_bit = false;
					Active_list[i].load_flag = false;
					Active_list[i].store_flag = false;
					Active_list[i].branch_flag = false;
					Active_list[i].amo_flag = false;
					Active_list[i].csr_flag = false;
					Active_list[i].PC = 'x';
				}
			
					
				for (int i = 0; i < tail_active_list ; i++)
				{
					Active_list[i].dest_flag = false;
					Active_list[i].log_reg_num_dest = 'x';
					Active_list[i].phys_reg_num_dest = 'x';
					Active_list[i].completed_bit = false ;
					Active_list[i].exception_bit = false;
					Active_list[i].load_violation_bit = false;
					Active_list[i].branch_misprediction_bit = false;
					Active_list[i].value_misprediction_bit = false;
					Active_list[i].load_flag = false;
					Active_list[i].store_flag = false;
					Active_list[i].branch_flag = false;
					Active_list[i].amo_flag = false;
					Active_list[i].csr_flag = false;
					Active_list[i].PC = 'x';
				}
			}
			
			
			
			head_free_list = Branch_Checkpoint[branch_ID].Checkpointed_Free_list_head_index;
			
			if(head_free_list >= tail_free_list)
				size_free_entry_free_list = head_free_list - tail_free_list;
			else
				size_free_entry_free_list = size_free_list + head_free_list - tail_free_list;

			

			if(tail_active_list == AL_index)
				size_free_entry_active_list = size_active_list - 1;
			else if(tail_active_list > AL_index)
				size_free_entry_active_list = size_free_entry_active_list + (tail_active_list - AL_index) -1;
			else
			{
				size_free_entry_active_list = size_free_entry_active_list + (size_active_list - AL_index + tail_active_list)-1;
			}
			

			if(AL_index == size_active_list-1)
			{
				tail_active_list = 0;
			}
			else
			{
				tail_active_list = AL_index + 1;
			}
		
		}
		//printf("\nRESOLVE AFter: Head  %d and Tail %d and Free entry Active List %d",head_active_list, tail_active_list,size_free_entry_active_list);

	}

	//////////////////////////////////////////
	// Functions related to Retire Stage.   //
	//////////////////////////////////////////

	///////////////////////////////////////////////////////////////////
	// This function allows the caller to examine the instruction at the head
	// of the Active List.
	//
	// Input arguments: none.
	//
	// Return value:
	// * Return "true" if the Active List is NOT empty, i.e., there
	//   is an instruction at the head of the Active List.
	// * Return "false" if the Active List is empty, i.e., there is
	//   no instruction at the head of the Active List.
	//
	// Output arguments:
	// Simply return the following contents of the head entry of
	// the Active List.  These are don't-cares if the Active List
	// is empty (you may either return the contents of the head
	// entry anyway, or not set these at all).
	// * completed bit
	// * exception bit
	// * load violation bit
	// * branch misprediction bit
	// * value misprediction bit
	// * load flag (indicates whether or not the instr. is a load)
	// * store flag (indicates whether or not the instr. is a store)
	// * branch flag (indicates whether or not the instr. is a branch)
	// * amo flag (whether or not instr. is an atomic memory operation)
	// * csr flag (whether or not instr. is a system instruction)
	// * program counter of the instruction
	/////////////////////////////////////////////////////////////////////
	bool renamer::precommit(bool &completed,
                       bool &exception, bool &load_viol, bool &br_misp, bool &val_misp,
	               bool &load, bool &store, bool &branch, bool &amo, bool &csr,
		       uint64_t &PC)
	{
		//printf("PRECommit : Head %d Tail %d Active List and Free entries in Active List %d\n", head_active_list, tail_active_list, size_free_entry_active_list);
			exception = Active_list[head_active_list].exception_bit;
			completed = Active_list[head_active_list].completed_bit;
			load_viol = Active_list[head_active_list].load_violation_bit;
			br_misp = Active_list[head_active_list].branch_misprediction_bit;
			val_misp = Active_list[head_active_list].value_misprediction_bit;
			load = Active_list[head_active_list].load_flag;
			store = Active_list[head_active_list].store_flag;
			branch = Active_list[head_active_list].branch_flag;
			amo = Active_list[head_active_list].amo_flag;
			csr = Active_list[head_active_list].csr_flag;
		//	if(Active_list[head_active_list].exception_bit == true)
			PC = Active_list[head_active_list].PC;
			
		
		if(size_active_list - size_free_entry_active_list > 0) // Not EMPTY
		{
			return true; 
		}
		else
		{
			return false;
		}
	}

	/////////////////////////////////////////////////////////////////////
	// This function commits the instruction at the head of the Active List.
	//
	// Tip (optional but helps catch bugs):
	// Before committing the head instruction, assert that it is valid to
	// do so (use assert() from standard library). Specifically, assert
	// that all of the following are true:
	// - there is a head instruction (the active list isn't empty)
	// - the head instruction is completed
	// - the head instruction is not marked as an exception
	// - the head instruction is not marked as a load violation
	// It is the caller's (pipeline's) duty to ensure that it is valid
	// to commit the head instruction BEFORE calling this function
	// (by examining the flags returned by "precommit()" above).
	// This is why you should assert() that it is valid to commit the
	// head instruction and otherwise cause the simulator to exit.
	/////////////////////////////////////////////////////////////////////
	void renamer::commit()
	{
		//printf("Commit\n");
		assert (size_active_list - size_free_entry_active_list > 0);
		assert (Active_list[head_active_list].completed_bit == true);
		assert (Active_list[head_active_list].exception_bit == false);
		assert (Active_list[head_active_list].load_violation_bit == false);
		assert (Active_list[head_active_list].branch_misprediction_bit == false);
		assert (Active_list[head_active_list].value_misprediction_bit == false);
		
		//precommit();

		
		int AMT_index;
	
		if(Active_list[head_active_list].dest_flag == true)
		{
			AMT_index = Active_list[head_active_list].log_reg_num_dest;
			// Check if Free List isn't full
			assert (size_free_entry_free_list > 0);
			
			Free_list[tail_free_list] = AMT[AMT_index];
			
			AMT[AMT_index] = Active_list[head_active_list].phys_reg_num_dest;

			size_free_entry_free_list--;

			if(tail_free_list == size_free_list -1)
			{
				tail_free_list =0;
			}
			else
			{
				tail_free_list++;
			}
		
		}
			
			Active_list[head_active_list].dest_flag = false;
			//Active_list[head_active_list].dest_flag = 'x';
			Active_list[head_active_list].log_reg_num_dest = 'x';
			Active_list[head_active_list].phys_reg_num_dest = 'x';
			Active_list[head_active_list].completed_bit = false;
			Active_list[head_active_list].exception_bit = false;
			Active_list[head_active_list].load_violation_bit = false;
			Active_list[head_active_list].branch_misprediction_bit = false;
			Active_list[head_active_list].value_misprediction_bit = false;
			Active_list[head_active_list].load_flag = false;
			Active_list[head_active_list].store_flag = false;
			Active_list[head_active_list].branch_flag = false;
			Active_list[head_active_list].amo_flag = false;
			Active_list[head_active_list].csr_flag = false;
			Active_list[head_active_list].PC = 'x';
		
		
		
		if(head_active_list == size_active_list - 1)
		{
			head_active_list =0;
			size_free_entry_active_list ++;
		}
		else
		{
			head_active_list++;
			size_free_entry_active_list ++;
		}
		
		//printf("Commit : Head %d Tail %d Active List and Free entries in Active List %d", head_active_list, tail_active_list, size_free_entry_active_list);

		


	}

	//////////////////////////////////////////////////////////////////////
	// Squash the renamer class.
	//
	// Squash all instructions in the Active List and think about which
	// sructures in your renamer class need to be restored, and how.
	//
	// After this function is called, the renamer should be rolled-back
	// to the committed state of the machine and all renamer state
	// should be consistent with an empty pipeline.
	/////////////////////////////////////////////////////////////////////
	void renamer::squash()
	{
		////printf("Squash\n");
		for (int i = 0 ; i < n_log_regs ; i++)
		{
			RMT[i] = AMT[i];
		}
		tail_active_list = head_active_list = 0;
		head_free_list = tail_free_list;
		
		size_free_entry_free_list = 0;
		size_free_entry_active_list = size_active_list;

		GBM = 0;
		
		for (int i =0 ; i < n_branches ; i++)
		{
			Branch_Checkpoint[i].Checkpointed_GBM = 0;
			Branch_Checkpoint[i].Checkpointed_Free_list_head_index = 0;
			for (int j = 0; j< n_log_regs; j++)
			{
				Branch_Checkpoint[i].Shadow_Map_Table[j] = 0;	
			
			}	
		}
			
		for (int i=0; i < size_active_list ; i++)
		{	
			Active_list[i].dest_flag = false;
			Active_list[i].log_reg_num_dest = 'x';
			Active_list[i].phys_reg_num_dest = 'x';
			Active_list[i].completed_bit = 'x';
			Active_list[i].exception_bit = 'x';
			Active_list[i].load_violation_bit = 'x';
			Active_list[i].branch_misprediction_bit = 'x';
			Active_list[i].value_misprediction_bit = 'x';
			Active_list[i].load_flag = 'x';
			Active_list[i].store_flag = 'x';
			Active_list[i].branch_flag = 'x';
			Active_list[i].amo_flag = 'x';
			Active_list[i].csr_flag = 'x';
			Active_list[i].PC = 'x';
		}
	

	}

	//////////////////////////////////////////
	// Functions not tied to specific stage.//
	//////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////
	// Functions for individually setting the exception bit,
	// load violation bit, branch misprediction bit, and
	// value misprediction bit, of the indicated entry in the Active List.
	/////////////////////////////////////////////////////////////////////
	void renamer::set_exception(uint64_t AL_index)
	{
		Active_list[AL_index].exception_bit = true;
	}
	void renamer::set_load_violation(uint64_t AL_index)
	{
		Active_list[AL_index].load_violation_bit = true;
	}
	void renamer::set_branch_misprediction(uint64_t AL_index)
	{
		Active_list[AL_index].branch_misprediction_bit = true;
	}
	void renamer::set_value_misprediction(uint64_t AL_index)
	{
		Active_list[AL_index].value_misprediction_bit = true;
	}

	/////////////////////////////////////////////////////////////////////
	// Query the exception bit of the indicated entry in the Active List.
	/////////////////////////////////////////////////////////////////////
	bool renamer::get_exception(uint64_t AL_index)
	{
		return Active_list[AL_index].exception_bit;
	}
