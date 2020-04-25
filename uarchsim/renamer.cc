#include <inttypes.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "renamer.h"
using namespace std;

renamer::renamer(uint64_t n_log_regs,
		uint64_t n_phys_regs,
		uint64_t n_branches)
{
	//std::cout<< "Constructor start "<< n_branches << "\n";
	assert(n_phys_regs > n_log_regs);
	assert(1 <= n_branches);
	assert(64 >= n_branches);
	uint64_t n_fl_regs, i;
	n_fl_regs = n_phys_regs - n_log_regs;
	RMT = new uint64_t[n_log_regs];
	for (i = 0; i < n_log_regs; i++)
		RMT[i] = i;
	AMT = new uint64_t[n_log_regs];
	for (i = 0; i < n_log_regs; i++)
		AMT[i] = i;
	FL = new uint64_t[n_fl_regs];
	for (i = 0; i < n_fl_regs; i++)
		FL[i] = i + n_log_regs;
	FL_head = 0;
	FL_tail = 0;
	set_FL_full();
//TODO indicate FL is full, implement FIFO
	AL = new active_list[n_fl_regs];
	AL_head = 0;
	AL_tail = 0;
	set_AL_empty();
	PRF = new uint64_t[n_phys_regs];
	PRF_ready = new bool[n_phys_regs];
	for (i = 0; i < n_phys_regs; i++)
		PRF[i] = 0;
	for (i = 0; i < n_phys_regs; i++)
		PRF_ready[i] = true;
	GBM = 0;
	BC = new branch_checkpoint[n_branches];
	for (i = 0; i < n_branches; i++)
		BC[i].RMT = new uint64_t[n_log_regs];
	log_regs = n_log_regs;
	phy_regs = n_phys_regs;
	fl_regs = n_fl_regs;
	branches = n_branches;
}

bool renamer::stall_reg (uint64_t bundle_dst)
{
	//std::cout<< "stall_reg start ";
	uint64_t i, cnt;
	cnt = 0;
	i = FL_head;
	while (i != FL_tail) {
		cnt++;
		i = (i + 1) % fl_regs;
	}
	//std::cout << FL_full << "," << bundle_dst << "," << cnt << "\n";
	if (FL_full)
		return false;
	else if (bundle_dst <= cnt)
		return false;
	else
		return true;
}

bool renamer::stall_branch (uint64_t bundle_branch)
{
	//std::cout<< "stall_branch start ";
	uint64_t i, j, cnt;
	j = 1;
	cnt = 0;
	for (i = 0; i < branches; i++) {
		if (((GBM >> i) & j) == 0)
			cnt++;
	}
	//std::cout << bundle_branch << "," << cnt << "\n";
	if (bundle_branch <= cnt)
		return false;
	else
		return true;
}

uint64_t renamer::get_branch_mask ()
{
	//std::cout<< "get_branch_mask start \n";
	return GBM;
}

uint64_t renamer::rename_rsrc (uint64_t log_reg)
{
	//std::cout<< "rename_rsrc start "<< log_reg << "," << RMT[log_reg] << "\n";
	return RMT[log_reg];
}

uint64_t renamer::rename_rdst (uint64_t log_reg)
{
	//std::cout<< "rename_rdst start " << log_reg << ",";
	RMT[log_reg] = FL[FL_head];
	FL_head = (FL_head + 1) % fl_regs;
	reset_FL_full();
	//std::cout << RMT[log_reg]<< "\n";
	return RMT[log_reg];
}

uint64_t renamer::checkpoint ()
{
	//std::cout<< "checkpoint start "<< //std::hex << GBM;
	uint64_t i, j, k, pos;
	j = 1;
	pos = branches;
	for (i = 0; i < branches; i++) {
		if (((GBM >> i) & 1) == 0) {
			pos = i;
			break;
		}
	}
	assert(pos != branches);
	for (i = 0; i < log_regs; i++)
		BC[pos].RMT[i] = RMT[i];
	BC[pos].FL_head = FL_head;
	BC[pos].GBM = GBM;
	k = j << pos;
	GBM = GBM | k;
	//std::cout << "," << GBM << "," << k << //std::dec << "," << pos<< "\n";
	return pos;
//	return 0;
}

bool renamer::stall_dispatch (uint64_t bundle_inst)
{
	//std::cout<< "stall_dispatch start";
	uint64_t i, cnt;
	i = AL_tail;
	cnt = 0;
	while (i != AL_head) {
		cnt++;
		i = (i + 1) % fl_regs;
	}
	//std::cout<< "al head "<< AL_head<<" tail "<< AL_tail << " empty "<<AL_empty<< " bundle_inst "<<bundle_inst <<" cnt available" <<cnt << "\n";
	if (AL_empty)
		return false;
	else if (bundle_inst <= cnt)
		return false;
	else
		return true;
	
}

uint64_t renamer::dispatch_inst (bool dest_valid,
			uint64_t log_reg,
			uint64_t phys_reg,
			bool load,
			bool store,
			bool branch,
			bool amo,
			bool csr,
			uint64_t PC)
{
	//std::cout<< "dispatch_inst start \n";
	uint64_t index;
	AL[AL_tail].dst_flag = dest_valid;
	AL[AL_tail].dest_l_reg = log_reg;
	AL[AL_tail].dest_p_reg = phys_reg;
	AL[AL_tail].comp = false;
	AL[AL_tail].excp = false;
	AL[AL_tail].ld_vio = false;
	AL[AL_tail].br_mis = false;
	AL[AL_tail].val_mis = false;
	AL[AL_tail].ld_flag = load;
	AL[AL_tail].st_flag = store;
	AL[AL_tail].br_flag = branch;
	AL[AL_tail].amo_flag = amo;
	AL[AL_tail].csr_flag = csr;
	AL[AL_tail].PC = PC;
	index = AL_tail;
	AL_tail = (AL_tail + 1) % fl_regs;
	//AL_empty = false;
	reset_AL_empty();
	//std::cout <<AL[index].dst_flag << "," << dest_valid<< "," <<AL[index].dest_l_reg << "," << log_reg<< "," <<	AL[index].dest_p_reg << "," << phys_reg<< "," <<AL[index].comp << "," <<AL[index].excp << "," <<AL[index].ld_vio << "," <<AL[index].br_mis << "," <<AL[index].val_mis << "," <<AL[index].ld_flag << "," << load<< "," <<AL[index].st_flag << "," << store<< "," <<AL[index].br_flag << "," << branch<< "," <<AL[index].amo_flag << "," << amo<< "," <<AL[index].csr_flag << "," << csr<< "," <<AL[index].PC << "," << PC<< "," <<index << "," << AL_tail<< "," <<AL_empty << "\n"; 
	return index;
}	

bool renamer::is_ready (uint64_t phys_reg)
{
	//std::cout<< "is_ready start "<<  PRF_ready[phys_reg] << "\n";
	return PRF_ready[phys_reg];
}

void renamer::clear_ready (uint64_t phys_reg)
{
	//std::cout<< "clear_ready start \n";
	PRF_ready[phys_reg] = false;
}

void renamer::set_ready (uint64_t phys_reg)
{
	//std::cout<< "set_ready start "<< phys_reg <<"\n";
	PRF_ready[phys_reg] = true;
}

uint64_t renamer::read (uint64_t phys_reg)
{
	//std::cout<< "read start \n";
	return PRF[phys_reg];
}

void renamer::write (uint64_t phys_reg, uint64_t value)
{
	//std::cout<< "write start "<< "," << value << "," << phys_reg << "," << PRF[phys_reg];
	PRF[phys_reg] = value;
	//std::cout<< "," << PRF[phys_reg] << "\n";
}

void renamer::set_complete (uint64_t AL_index)
{
	//std::cout<< "set_complete start \n";
	AL[AL_index].comp = true;
}

void renamer::resolve (	uint64_t AL_index,
			uint64_t branch_ID,
			bool correct)
{
	//std::cout<< "resolve start "<< correct <<","<< //std::hex << GBM;
	uint64_t i, j, branch_pos, not_branch_pos;
	j = 1;
	branch_pos = j << branch_ID;
	not_branch_pos = ~branch_pos;
	if (correct == true) {
		GBM = GBM & (~branch_pos);
		for (i = 0; i < branches; i++)
			BC[i].GBM = BC[i].GBM & not_branch_pos;
	}
	else {
		GBM = BC[branch_ID].GBM;
		for (i = 0; i < branches; i++)
			BC[i].GBM = BC[i].GBM & not_branch_pos;
		FL_head = BC[branch_ID].FL_head;
		if (FL_head == FL_tail)
			set_FL_full();
		for (i = 0; i < log_regs; i++)
			RMT[i] = BC[branch_ID].RMT[i];
		AL_tail = (AL_index + 1) % fl_regs;
		if (AL_head == AL_tail)
			reset_AL_empty();
	}
	//std::cout << "," << GBM << "," << branch_pos << "," << branch_ID << std::dec<< " AL_tail " << AL_tail <<"\n";
}

bool renamer::precommit (	bool &completed,
			bool &exception, bool &load_viol, bool &br_misp, bool &val_misp,
			bool &load, bool &store, bool &branch, bool &amo, bool &csr,
			uint64_t &PC)
{// TODO check if this works
	//std::cout<< "precommit start \n";
	completed = AL[AL_head].comp;
	exception = AL[AL_head].excp;
	load_viol = AL[AL_head].ld_vio;
	br_misp = AL[AL_head].br_mis;
	val_misp = AL[AL_head].val_mis;
	load = AL[AL_head].ld_flag;
	store = AL[AL_head].st_flag;
	branch = AL[AL_head].br_flag;
	amo = AL[AL_head].amo_flag;
	csr = AL[AL_head].csr_flag;
	PC = AL[AL_head].PC;
	//std::cout << AL[AL_head].comp<<","<< AL[AL_head].excp<<","<< AL[AL_head].ld_vio<<","<< AL[AL_head].br_mis<<","<< AL[AL_head].val_mis <<","<< AL[AL_head].ld_flag<< AL[AL_head].st_flag<< AL[AL_head].br_flag<<","<< AL[AL_head].amo_flag<<","<< AL[AL_head].csr_flag<<","<< AL[AL_head].PC << "," << AL_empty << "\n";
	//std::cout << completed <<","<< AL[AL_head].comp<<","<< exception <<","<< AL[AL_head].excp<<","<< load_viol <<","<< AL[AL_head].ld_vio<<","<< br_misp <<","<< AL[AL_head].br_mis<<","<< val_misp <<","<< AL[AL_head].val_mis<<","<< load <<","<< AL[AL_head].ld_flag<<","<< store <<","<< AL[AL_head].st_flag<<","<< branch <<","<< AL[AL_head].br_flag<<","<< amo <<","<< AL[AL_head].amo_flag<<","<< csr <<","<< AL[AL_head].csr_flag<<","<< PC <<","<< AL[AL_head].PC << "," << AL_empty << "\n";
	if (AL_empty)
		return false;
	else
		return true;
}

void renamer::commit ()
{
	//std::cout<< "commit start ";
	assert(!AL_empty);
	assert(AL[AL_head].comp);
	assert(!AL[AL_head].excp);
	assert(!AL[AL_head].ld_vio);
	if (AL[AL_head].dst_flag) {
	//std::cout<< "dst " <<  AL[AL_head].dest_l_reg << "," << AL[AL_head].dest_p_reg <<","<< PRF[AL[AL_head].dest_p_reg];
		//PRF_ready[AMT[AL[AL_head].dest_l_reg]] = false;
		FL[FL_tail] = AMT[AL[AL_head].dest_l_reg];
		AMT[AL[AL_head].dest_l_reg] = AL[AL_head].dest_p_reg;
		FL_tail = (FL_tail + 1) % fl_regs;
		if (FL_head == FL_tail)
			set_FL_full();
	}
	//std::cout <<"\n";
	AL_head = (AL_head + 1) % fl_regs;
	if (AL_head == AL_tail)
		set_AL_empty();
}

void renamer::squash()
{
	//std::cout<< "squash start \n";
	uint64_t i;
	for (i = 0; i < log_regs; i++)
		RMT[i] = AMT[i];
	for (i = 0; i < log_regs; i++)
		PRF_ready[AMT[i]] = true;
	FL_head = FL_tail;
	set_FL_full();
	AL_tail = AL_head;
	set_AL_empty();
	GBM = 0;
}

void renamer::set_exception(uint64_t AL_index)
{
	//std::cout<< "set_exception start \n";
	AL[AL_index].excp = true;
}

void renamer::set_load_violation(uint64_t AL_index)
{
	//std::cout<< "set_load_violation start \n";
	AL[AL_index].ld_vio = true;
}

void renamer::set_branch_misprediction(uint64_t AL_index)
{
	//std::cout<< "set_branch_misprediction start \n";
	AL[AL_index].br_mis = true;
}

void renamer::set_value_misprediction(uint64_t AL_index)
{
	//std::cout<< "set_value_misprediction start \n";
	AL[AL_index].val_mis = true;
}

bool renamer::get_exception(uint64_t AL_index)
{
	//std::cout<< "get_exception start \n";
	return AL[AL_index].excp;
}

void renamer::set_AL_empty ()
{
	AL_empty = true;
}

void renamer::reset_AL_empty ()
{
	AL_empty = false;
}

void renamer::set_FL_full ()
{
	FL_full = true;
}

void renamer::reset_FL_full ()
{
	FL_full = false;
}
