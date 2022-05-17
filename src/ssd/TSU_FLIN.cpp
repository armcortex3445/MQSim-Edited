#include "TSU_FLIN.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include <stack>
#include <cmath>
#include <vector>
#include "TSU_Priority_OutOfOrder.h"
#include "Host_Interface_Defs.h"
#include <assert.h>


/*
* TSU_FLIN class is modified by Seong Jihwan to successfully simulate FLIN Algorithm implemented till 2nd stage. 
* FLIN Algorithm is introduced by A. Tavakkol et al ,ISCA, 2018.
* Modified Point 
* - MQ Sim Can Use partial FLIN Algorithm which is implemented till 2nd stage .
*
*/
namespace SSD_Components
{

	//TSU_FLIN Creator 
	TSU_FLIN::TSU_FLIN(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController,
		const unsigned int channel_count, const unsigned int chip_no_per_channel, const unsigned int die_no_per_chip, const unsigned int plane_no_per_die, unsigned int flash_page_size,
		const sim_time_type flow_classification_epoch, const unsigned int alpha_read, const unsigned int alpha_write,
		const unsigned int no_of_priority_classes, const stream_id_type max_flow_id, const unsigned int* stream_count_per_priority_class, stream_id_type** stream_ids_per_priority_class, const double f_thr,
		const sim_time_type WriteReasonableSuspensionTimeForRead,
		const sim_time_type EraseReasonableSuspensionTimeForRead,
		const sim_time_type EraseReasonableSuspensionTimeForWrite,
		const bool EraseSuspensionEnabled, const bool ProgramSuspensionEnabled)
		:
	
	TSU_Base(id, ftl, NVMController, Flash_Scheduling_Type::FLIN, channel_count, chip_no_per_channel, die_no_per_chip, plane_no_per_die,
		WriteReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForWrite,
		EraseSuspensionEnabled, ProgramSuspensionEnabled),

		flow_classification_epoch(flow_classification_epoch),
		no_of_priority_classes(no_of_priority_classes), F_thr(f_thr)
	{
		alpha_read_for_epoch = alpha_read / (channel_count * chip_no_per_channel) / flash_page_size;	
		alpha_write_for_epoch = alpha_write / (channel_count * chip_no_per_channel) / flash_page_size,	
			this->stream_count_per_priority_class = new unsigned int[no_of_priority_classes];
		this->stream_ids_per_priority_class = new stream_id_type * [no_of_priority_classes];
		for (unsigned int i = 0; i < no_of_priority_classes; i++)
		{	
			this->stream_count_per_priority_class[i] = stream_count_per_priority_class[i];
			this->stream_ids_per_priority_class[i] = new stream_id_type[stream_count_per_priority_class[i]];
			for (stream_id_type stream_cntr = 0; stream_cntr < stream_count_per_priority_class[i]; stream_cntr++)
				this->stream_ids_per_priority_class[i][stream_cntr] = stream_ids_per_priority_class[i][stream_cntr];
		}


		UserReadTRQueue = new Flash_Transaction_Queue * *[channel_count];
		UserWriteTRQueue = new Flash_Transaction_Queue * *[channel_count];
		GCReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCWriteTRQueue = new Flash_Transaction_Queue * [channel_count];
		GCEraseTRQueue = new Flash_Transaction_Queue * [channel_count];
		MappingReadTRQueue = new Flash_Transaction_Queue * [channel_count];
		MappingWriteTRQueue = new Flash_Transaction_Queue * [channel_count];
		flow_activity_info = new FLIN_Flow_Monitoring_Unit * **[channel_count];

		low_intensity_class_read = new std::set<stream_id_type>**[channel_count];
		low_intensity_class_write = new std::set<stream_id_type>**[channel_count];
		
 
		head_high_read = new Flash_Transaction_Queue::iterator * *[channel_count];
		head_high_write = new Flash_Transaction_Queue::iterator * *[channel_count];
		
		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			UserReadTRQueue[channel_id] = new Flash_Transaction_Queue * [chip_no_per_channel];
			UserWriteTRQueue[channel_id] = new Flash_Transaction_Queue * [chip_no_per_channel];
			GCReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			GCWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			GCEraseTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			MappingReadTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			MappingWriteTRQueue[channel_id] = new Flash_Transaction_Queue[chip_no_per_channel];
			flow_activity_info[channel_id] = new FLIN_Flow_Monitoring_Unit * *[chip_no_per_channel];
			low_intensity_class_read[channel_id] = new std::set<stream_id_type>*[chip_no_per_channel];
			low_intensity_class_write[channel_id] = new std::set<stream_id_type>*[chip_no_per_channel];

			head_high_read[channel_id] = new Flash_Transaction_Queue::iterator * [chip_no_per_channel];
			head_high_write[channel_id] = new Flash_Transaction_Queue::iterator * [chip_no_per_channel];
			
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				head_high_read[channel_id][chip_id] = new Flash_Transaction_Queue::iterator[no_of_priority_classes];
				head_high_write[channel_id][chip_id] = new Flash_Transaction_Queue::iterator[no_of_priority_classes];
				UserReadTRQueue[channel_id][chip_id] = new Flash_Transaction_Queue[no_of_priority_classes];
				UserWriteTRQueue[channel_id][chip_id] = new Flash_Transaction_Queue[no_of_priority_classes];
				flow_activity_info[channel_id][chip_id] = new FLIN_Flow_Monitoring_Unit * [no_of_priority_classes];
				low_intensity_class_read[channel_id][chip_id] = new std::set<stream_id_type>[no_of_priority_classes];
				low_intensity_class_write[channel_id][chip_id] = new std::set<stream_id_type>[no_of_priority_classes];

				GCReadTRQueue[channel_id][chip_id].Set_id("GC_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@");
				MappingReadTRQueue[channel_id][chip_id].Set_id("Mapping_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				MappingWriteTRQueue[channel_id][chip_id].Set_id("Mapping_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCWriteTRQueue[channel_id][chip_id].Set_id("GC_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCEraseTRQueue[channel_id][chip_id].Set_id("GC_Erase_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				for (unsigned int pclass_id = 0; pclass_id < no_of_priority_classes; pclass_id++)
				{
					flow_activity_info[channel_id][chip_id][pclass_id] = new FLIN_Flow_Monitoring_Unit[max_flow_id];
					for (int stream_cntr = 0; stream_cntr < max_flow_id; stream_cntr++)
					{
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_read_requests_history = 0;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_read_requests_recent = 0;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_write_requests_history = 0;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_write_requests_recent = 0;

						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_read_requests_total = 0;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_write_requests_total = 0;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Sum_RD_TR_alone_turnaround_time = 1;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Sum_RD_TR_shared_turnaround_time = 1;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Sum_WR_TR_alone_turnaround_time = 1;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Sum_WR_TR_shared_turnaround_time = 1;
					}
					UserReadTRQueue[channel_id][chip_id][pclass_id].Set_id("User_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@" + std::to_string(pclass_id));
					UserWriteTRQueue[channel_id][chip_id][pclass_id].Set_id("User_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@" + std::to_string(pclass_id));
					head_high_read[channel_id][chip_id][pclass_id] = UserReadTRQueue[channel_id][chip_id][pclass_id].end();
					head_high_write[channel_id][chip_id][pclass_id] = UserWriteTRQueue[channel_id][chip_id][pclass_id].end();
				}
			}
		}

	}

	TSU_FLIN::~TSU_FLIN()
	{
		delete[] this->stream_count_per_priority_class;

		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				delete[] UserReadTRQueue[channel_id][chip_id];
				delete[] UserWriteTRQueue[channel_id][chip_id];
				delete[] low_intensity_class_read[channel_id][chip_id];
				delete[] low_intensity_class_write[channel_id][chip_id];

				for (unsigned int pclass_id = 0; pclass_id < no_of_priority_classes; pclass_id++)
				{
					delete[] flow_activity_info[channel_id][chip_id][pclass_id];
				}
				delete[] flow_activity_info[channel_id][chip_id];
				delete[] head_high_read[channel_id][chip_id];
				delete[] head_high_write[channel_id][chip_id];
			}

			delete[] UserReadTRQueue[channel_id];
			delete[] UserWriteTRQueue[channel_id];
			delete[] GCReadTRQueue[channel_id];
			delete[] GCWriteTRQueue[channel_id];
			delete[] GCEraseTRQueue[channel_id];
			delete[] MappingReadTRQueue[channel_id];
			delete[] MappingWriteTRQueue[channel_id];
			delete[] low_intensity_class_read[channel_id];
			delete[] low_intensity_class_write[channel_id];

			delete[] head_high_read[channel_id];
			delete[] head_high_write[channel_id];
			delete[] flow_activity_info[channel_id];
		}
		delete[] UserReadTRQueue;
		delete[] UserWriteTRQueue;
		delete[] GCReadTRQueue;
		delete[] GCWriteTRQueue;
		delete[] GCEraseTRQueue;
		delete[] MappingReadTRQueue;
		delete[] MappingWriteTRQueue;
		delete[] low_intensity_class_read;
		delete[] low_intensity_class_write;
		delete[] head_high_read;
		delete[] head_high_write;
		delete[] flow_activity_info;
	}

	// initial setting of TSU_FLIN instance on simulation.
	void TSU_FLIN::Start_simulation()
	{
		std::string* parameter = new std::string("FLIN");
		Simulator->Register_sim_event(flow_classification_epoch, this, parameter, 0);
	}

	// TSU_FLIN instance don't need this 
	void TSU_FLIN::Validate_simulation_config() {}

	// during predetermined interval, FLIN Algorithm classifies Flow I/O intensity, as described in section 5.1 in FLIN paper
	void TSU_FLIN::Execute_simulator_event(MQSimEngine::Sim_Event* event)
	{
		//Flow classification as described in Section 5.1 of FLIN paper in ISCA 2018

		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				for (unsigned int pclass_id = 0; pclass_id < no_of_priority_classes; pclass_id++)
				{
					if (stream_count_per_priority_class[pclass_id] < 2)
						continue;

					low_intensity_class_read[channel_id][chip_id][pclass_id].clear();
					low_intensity_class_write[channel_id][chip_id][pclass_id].clear();
					for (unsigned int stream_cntr = 0; stream_cntr < stream_count_per_priority_class[pclass_id]; stream_cntr++)
					{
						if (flow_activity_info[channel_id][chip_id][pclass_id][stream_ids_per_priority_class[pclass_id][stream_cntr]].Serviced_read_requests_recent < alpha_read_for_epoch)
							low_intensity_class_read[channel_id][chip_id][pclass_id].insert(stream_ids_per_priority_class[pclass_id][stream_cntr]);
						if (flow_activity_info[channel_id][chip_id][pclass_id][stream_ids_per_priority_class[pclass_id][stream_cntr]].Serviced_write_requests_recent < alpha_write_for_epoch)
							low_intensity_class_write[channel_id][chip_id][pclass_id].insert(stream_ids_per_priority_class[pclass_id][stream_cntr]);

						flow_activity_info[channel_id][chip_id][pclass_id][stream_ids_per_priority_class[pclass_id][stream_cntr]].Serviced_read_requests_history += flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_read_requests_recent;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_ids_per_priority_class[pclass_id][stream_cntr]].Serviced_read_requests_recent = 0;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_ids_per_priority_class[pclass_id][stream_cntr]].Serviced_write_requests_history += flow_activity_info[channel_id][chip_id][pclass_id][stream_cntr].Serviced_write_requests_recent;
						flow_activity_info[channel_id][chip_id][pclass_id][stream_ids_per_priority_class[pclass_id][stream_cntr]].Serviced_write_requests_recent = 0;
					}
				}
			}
		}
		
		// if Host no more create IO request, MQ Sim finishes simulation  
		if (Simulator->Get_sim_event_count() == 1 &&event->Next_event==NULL)
		{
			std::cout << "FLIN Sim event loop occured" << std::endl;
		}
		else {
			Simulator->Register_sim_event(Simulator->Time() + flow_classification_epoch, this, event->Parameters, 0);

		}
	}

	
	inline void TSU_FLIN::Prepare_for_transaction_submit()
	{
		opened_scheduling_reqs++; 
		if (opened_scheduling_reqs > 1)
			return;


		transaction_receive_slots.clear(); 
	}

	
	inline void TSU_FLIN::Submit_transaction(NVM_Transaction_Flash* transaction)
	{
		transaction_receive_slots.push_back(transaction);
	}

	// Scheduling transaction by using FLIN Algorithm, as described in section 4 in FLIN paper.
	// this FLIN Algorithm is patially implemented till 2 stage Priority-Aware Queue Arbitration. 
	void TSU_FLIN::Schedule()
	{
		opened_scheduling_reqs--;
		if (opened_scheduling_reqs > 0)
			return;
		if (opened_scheduling_reqs < 0)
			PRINT_ERROR("TSU Schedule function is invoked in an incorrect way!");


		if (transaction_receive_slots.size() == 0)
			return;


		for (std::list<NVM_Transaction_Flash*>::iterator it = transaction_receive_slots.begin();
			it != transaction_receive_slots.end(); it++)
		{

			flash_channel_ID_type channel_id = (*it)->Address.ChannelID;
			flash_chip_ID_type chip_id = (*it)->Address.ChipID;

			unsigned int priority_class;


			if ((*it)->Source == Transaction_Source_Type::USERIO || (*it)->Source == Transaction_Source_Type::CACHE)
			{
				priority_class = (int)(*it)->Priority_class;
				if (priority_class > 3)
				{
					if ((*it)->UserIORequest != (User_Request*)NULL) {
						priority_class = (int)(*it)->UserIORequest->Priority_class;
						(*it)->Priority_class = (*it)->UserIORequest->Priority_class;
						if (priority_class > 3)
						{
							priority_class = IO_Flow_Priority_Class::HIGH;
							(*it)->Priority_class = IO_Flow_Priority_Class::HIGH;
						}
					}
					else {
						priority_class = IO_Flow_Priority_Class::HIGH;
						(*it)->Priority_class = IO_Flow_Priority_Class::HIGH;
					}

				}
			}
			else {
				priority_class = 1;
			}

			stream_id_type stream_id = (*it)->Stream_id;

			switch ((*it)->Type)
			{

			case Transaction_Type::READ:
				switch ((*it)->Source)
				{

				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:

					(*it)->Enqueue_time = Simulator->Time();
					(*it)->Transfer_memory_time = _NVMController->Estimate_transfer_and_memory_time(*it);

					if (stream_count_per_priority_class[priority_class] < 2)
					{
						UserReadTRQueue[channel_id][chip_id][priority_class].push_back(*it);
						break;
					}
					flow_activity_info[channel_id][chip_id][priority_class][stream_id].Serviced_read_requests_recent+= (*it)->Data_and_metadata_size_in_byte;
					flow_activity_info[channel_id][chip_id][priority_class][stream_id].Serviced_read_requests_total += (*it)->Data_and_metadata_size_in_byte;

					/* 1 stage : Fairenss-Aware Queue Insertion , as decribed in section 4.1 of FLIN paper */

					// Step 1 : Prioritize Low-Intensity Flows.
					//if (new Transaction is from High read intensity Flow(stream) )
					if (low_intensity_class_read[channel_id][chip_id][priority_class].find(stream_id) == low_intensity_class_read[channel_id][chip_id][priority_class].end())
					{

						UserReadTRQueue[channel_id][chip_id][priority_class].push_back(*it);
						auto tail = UserReadTRQueue[channel_id][chip_id][priority_class].end();
						tail--;

						estimate_alone_waiting_time(&UserReadTRQueue[channel_id][chip_id][priority_class], tail);//EstimateAloneWaitingTime(Q, TRnew)

						//if head of high read transacions in Queue is NULL, updating head. 
						if (head_high_read[channel_id][chip_id][priority_class] == UserReadTRQueue[channel_id][chip_id][priority_class].end())
							head_high_read[channel_id][chip_id][priority_class] = prev(UserReadTRQueue[channel_id][chip_id][priority_class].end());

						// Step 2b : Maximize Fairness Among High-Intensity Flows 
						stream_id_type flow_with_max_average_slowdown;

						double f = fairness_based_on_average_slowdown(channel_id, chip_id, priority_class, true, flow_with_max_average_slowdown);

						if (f < F_thr && stream_id == flow_with_max_average_slowdown) {				
							move_forward(&UserReadTRQueue[channel_id][chip_id][priority_class],
								tail,
								head_high_read[channel_id][chip_id][priority_class]);//MoveForward(from Q.Tail up to Q.Taillow + 1)
						}

						else {
							reorder_for_fairness(&UserReadTRQueue[channel_id][chip_id][priority_class],
								head_high_read[channel_id][chip_id][priority_class],
								UserReadTRQueue[channel_id][chip_id][priority_class].end());//ReorderForFairness(from Q.Tail to Q.Taillow + 1)
						}
					}
					else
					{
						// Step 2a : Maximize Fairness Aming Low-Intensity Flows	
						UserReadTRQueue[channel_id][chip_id][priority_class].insert(head_high_read[channel_id][chip_id][priority_class], *it);//Insert(TRnew after Q.Taillow)
						reorder_for_fairness(&UserReadTRQueue[channel_id][chip_id][priority_class],
							UserReadTRQueue[channel_id][chip_id][priority_class].begin(), head_high_read[channel_id][chip_id][priority_class]);//ReorderForFairness(from Q.Taillow to Q.Head)
					}
					break;
				case Transaction_Source_Type::MAPPING:
					MappingReadTRQueue[channel_id][chip_id].push_back(*it);
					break;
				case Transaction_Source_Type::GC_WL:
					GCReadTRQueue[channel_id][chip_id].push_back(*it);
					break;
				default:
					PRINT_ERROR("TSU_OutOfOrder: Unhandled source type four read transaction!")
				}
				break;
			case Transaction_Type::WRITE:
				switch ((*it)->Source)
				{
				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:

					(*it)->Enqueue_time = Simulator->Time();
					(*it)->Transfer_memory_time = _NVMController->Estimate_transfer_and_memory_time(*it);

					if (stream_count_per_priority_class[priority_class] < 2)
					{
						UserWriteTRQueue[channel_id][chip_id][priority_class].push_back(*it);
						break;
					}
					flow_activity_info[channel_id][chip_id][priority_class][stream_id].Serviced_write_requests_recent += (*it)->Data_and_metadata_size_in_byte;
					flow_activity_info[channel_id][chip_id][priority_class][stream_id].Serviced_write_requests_total += (*it)->Data_and_metadata_size_in_byte;

					/* 1 stage : Fairenss-Aware Queue Insertion , as decribed in section 4.1 of FLIN paper */

					// Step 1 : Prioritize Low-Intensity Flows.
					//if (new Transaction is from High write intensity Flow(stream) )
					if (low_intensity_class_write[channel_id][chip_id][priority_class].find(stream_id) == low_intensity_class_write[channel_id][chip_id][priority_class].end())
					{
						UserWriteTRQueue[channel_id][chip_id][priority_class].push_back(*it);//Insert(TRnew after Q.Tail)
						auto tail = UserWriteTRQueue[channel_id][chip_id][priority_class].end();
						tail--;

						estimate_alone_waiting_time(&UserWriteTRQueue[channel_id][chip_id][priority_class], tail);//EstimateAloneWaitingTime(Q, TRnew)
						if (head_high_write[channel_id][chip_id][priority_class] == UserWriteTRQueue[channel_id][chip_id][priority_class].end())
							head_high_write[channel_id][chip_id][priority_class]=prev(UserWriteTRQueue[channel_id][chip_id][priority_class].end());

						stream_id_type flow_with_max_average_slowdown;
						double f = fairness_based_on_average_slowdown(channel_id, chip_id, priority_class, false, flow_with_max_average_slowdown);//FairnessBasedOnAverageSlowdown(Q)

						// Step 2b : Maximize Fairness Among High-Intensity Flows 
						if (f < F_thr && stream_id == flow_with_max_average_slowdown)
							move_forward(&UserWriteTRQueue[channel_id][chip_id][priority_class],
								tail,
								head_high_write[channel_id][chip_id][priority_class]);//MoveForward(from Q.Tail up to Q.Taillow + 1)
						else reorder_for_fairness(&UserWriteTRQueue[channel_id][chip_id][priority_class],
							head_high_write[channel_id][chip_id][priority_class],
							UserWriteTRQueue[channel_id][chip_id][priority_class].end());//ReorderForFairness(from Q.Tail to Q.Taillow + 1)
					}
					else
					{
						// Step 2a : Maximize Fairness Aming Low-Intensity Flows	
						UserWriteTRQueue[channel_id][chip_id][priority_class].insert(head_high_write[channel_id][chip_id][priority_class], *it);
						reorder_for_fairness(&UserWriteTRQueue[channel_id][chip_id][priority_class],
							UserWriteTRQueue[channel_id][chip_id][priority_class].begin(), head_high_write[channel_id][chip_id][priority_class]);
					}
					break;
				case Transaction_Source_Type::MAPPING:
					MappingWriteTRQueue[channel_id][chip_id].push_back(*it);
					break;
				case Transaction_Source_Type::GC_WL:
					GCWriteTRQueue[channel_id][chip_id].push_back(*it);
					break;
				default:
					PRINT_ERROR("TSU_OutOfOrder: Unhandled source type four write transaction!")
				}
				break;
			case Transaction_Type::ERASE:
				GCEraseTRQueue[channel_id][chip_id].push_back(*it);
				break;
			default:
				break;
			}
		}


		for (flash_channel_ID_type channel_id = 0; channel_id < channel_count; channel_id++)
		{
			if (_NVMController->Get_channel_status(channel_id) == BusChannelStatus::IDLE)
			{
				for (unsigned int i = 0; i < chip_no_per_channel; i++) {
					NVM::FlashMemory::Flash_Chip* chip = _NVMController->Get_chip(channel_id, Round_robin_turn_of_channel[channel_id]);
					//The TSU does not check if the chip is idle or not since it is possible to suspend a busy chip and issue a new command
					if (!service_read_transaction(chip))
						if (!service_write_transaction(chip))
							service_erase_transaction(chip);
					Round_robin_turn_of_channel[channel_id] = (flash_chip_ID_type)(Round_robin_turn_of_channel[channel_id] + 1) % chip_no_per_channel;
					if (_NVMController->Get_channel_status(chip->ChannelID) != BusChannelStatus::IDLE)
						break;
				}
			}
		}
	}


	// Edited Function for using FLIN Algorith. By Seong Jihwan
	// This Function Reorders Transaction from start to end -1 in Queue to maximize fairness of the Queued transactions.
	void TSU_FLIN::reorder_for_fairness(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator start, std::list<NVM_Transaction_Flash*>::iterator end)
	{

		if (start == prev(end) || start == end) {
			return;
		}

		//   Queue = [begin] [begin + 1] ... [start -1 ] [start] [start+1] ... [end-1] [NULL]
		//			   head  <----------------------------------------------------------> tail
		//			 begin() <----------------------------------------------------------> end()
		//   direction :itr ->

		std::list<NVM_Transaction_Flash*>::iterator itr = queue->begin();
		sim_time_type time_to_finish = 0;

		//Step 0 : calculating busy time of chip for estimating shared turnaround time of transaction , as described in 5.2 Section of FLIN paper.
		if (_NVMController->Is_chip_busy(*itr)) {
			if (_NVMController->Expected_finish_time(*itr) > Simulator->Time()) {
				time_to_finish = _NVMController->Expected_finish_time(*itr) - Simulator->Time();//T^chip_busy
			}
		}

		std::vector<double> slowdown_vector;
		int idx = 0;

		//Step 0 : calculating sum of (memory time + transfer time) of Transactions from begin position to previous start position, for estimating shared turnaround time of transaction , as described in 5.2 Section of FLIN paper.
		while (itr != start) {	
			time_to_finish += (*itr)->Transfer_memory_time;
			itr++;
		}




		//Step 1 : First pass - calcuating slowdown of each transaction, as decribed in 5.3 section of FLIN paper.
		double slowdown_min = 100000000000000000, slowdown_max = 0;


		while (itr != end) {
			sim_time_type T_TR_wait_shared = time_to_finish + (Simulator->Time() - (*itr)->Enqueue_time);

			estimate_alone_waiting_time(queue, itr);
			sim_time_type T_TR_wait_alone = (*itr)->Estimated_alone_waiting_time;

			double slowdown = (double)(T_TR_wait_shared + (*itr)->Transfer_memory_time) / (double)(T_TR_wait_alone + (*itr)->Transfer_memory_time);
			slowdown_vector.push_back(slowdown);

			if (slowdown < slowdown_min) {
				slowdown_min = slowdown;
			}
			if (slowdown > slowdown_max) {
				slowdown_max = slowdown;
			}
			time_to_finish += T_TR_wait_shared + (*itr)->Transfer_memory_time;
			idx++;
			itr++;
		}

		double fairness_max = slowdown_min / slowdown_max;

		//Step 1 : Second pass - traverling transaction from end - 1 to start to find a position for TR_new , that maximizes fairness, as decribed in 5.3 section of FLIN paper.

		auto final_position = prev(end); 
		auto position = prev(final_position);
		idx = idx - 2;
		int vector_size = idx+1;


		time_to_finish -= (*final_position)->Transfer_memory_time;

		sim_time_type T_new = (*final_position)->Transfer_memory_time;


		//   Queue = [begin] [begin + 1] ... [start -1 ] [start] [start+1] [end-2] [end-1] [NULL]
		//			   head  <---------------------------------------------------------------> tail
		//			 begin() <----------------------------------------------------------------> end()
		//																	<= idx 
		//																    <= position
		//																			<= final_position 


		sim_time_type T_pos_alone = 0;
		sim_time_type T_new_alone = (*final_position)->Estimated_alone_waiting_time;
		
		while (!(*position)->FLIN_Barrier)
		{
			reordering_loop_count++;
			time_to_finish -= (*position)->Transfer_memory_time;
			
			if ((*position)->Stream_id == (*prev(end))->Stream_id)
			{
				T_pos_alone = (*position)->Estimated_alone_waiting_time + T_new;
				T_new_alone = (*position)->Estimated_alone_waiting_time;
			}
			else {
				T_pos_alone = (*position)->Estimated_alone_waiting_time;
			}
			sim_time_type T_pos_shared = time_to_finish + T_new;
			sim_time_type T_new_shared = time_to_finish;

			double slowdown_pos = (double)(T_pos_shared + (*position)->Transfer_memory_time) / ((double)T_pos_alone + (*position)->Transfer_memory_time);

			double slowdown_new = (double)(T_new_shared + T_new) / (double)(T_new_alone + T_new);

			slowdown_vector[idx] = slowdown_new;
			slowdown_vector[idx + 1] = slowdown_pos;

			slowdown_min = 100000000000000000, slowdown_max = 0;

			for (int i = vector_size; i >=0; i--)
			{
				if (slowdown_vector[i] < slowdown_min)
					slowdown_min = slowdown_vector[i];
				if (slowdown_vector[i] > slowdown_max)
					slowdown_max = slowdown_vector[i];
			}


			double fairness_reorder = (double)slowdown_min / (double)slowdown_max;
			if (fairness_reorder > fairness_max)
			{
				fairness_max = fairness_reorder;
				final_position = position;
			}

			if (position == start)
				break;

			position--;
			idx--;


		}


		// Step 2 : moving new transaction to final position, as decribed in 5.3 section of FLIN paper. 
		if (final_position != prev(end))
		{ 
			NVM_Transaction_Flash* tr = *prev(end);
			queue->remove(prev(end));
			queue->insert(final_position, tr);

			itr = prev(end);
			//updating alone waiting time of transaction s
			while (itr != final_position)
			{
				estimate_alone_waiting_time(queue, itr);
				reordering_success_loop_count++;
				itr--;
			}
			estimate_alone_waiting_time(queue, itr);

			if (tr->Type == Transaction_Type::WRITE) {
				if (final_position == head_high_write[tr->Address.ChannelID][tr->Address.ChipID][tr->Priority_class]) {
					head_high_write[tr->Address.ChannelID][tr->Address.ChipID][tr->Priority_class] = prev(final_position);
				}
			}
			else {
				if (final_position == head_high_read[tr->Address.ChannelID][tr->Address.ChipID][tr->Priority_class]) {
					head_high_read[tr->Address.ChannelID][tr->Address.ChipID][tr->Priority_class] = prev(final_position);
				}
			}
			
		}

	}

	//Edited Function for using FLIN Algorithm. By Seong Jihwan
	//This function estimate alone waiting time of queued Transacion located at position.  
	sim_time_type TSU_FLIN::estimate_alone_waiting_time(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator position)
	{
		auto itr = position;
		sim_time_type expected_alone_waiting_time = 0;

		//process of estimate is decribed in section 5.2 In FLIN paper.

		// searching for last Transaction from flow of Tranascion located at position. 
		while (itr != queue->begin())
		{

			if ((*itr)->Stream_id == (*position)->Stream_id) {
				break;
			}
			itr--;		
		}

		if (itr == queue->begin())
		{
			if ((*itr)->Stream_id != (*position)->Stream_id || queue->size() == 1)
			{
				NVM_Transaction_Flash* chip_tr = _NVMController->Is_chip_busy_with_stream(*position);

				if (chip_tr == NULL)
				{

					(*position)->Estimated_alone_waiting_time = 0;
					return (*position)->Estimated_alone_waiting_time;
				}
				else
				{	
					if (_NVMController->Expected_finish_time(chip_tr) > Simulator->Time()) {
						expected_alone_waiting_time = _NVMController->Expected_finish_time(chip_tr) - Simulator->Time();
					}
					else {
						expected_alone_waiting_time = 0;
					}

					(*position)->Estimated_alone_waiting_time = expected_alone_waiting_time;

					return (*position)->Estimated_alone_waiting_time;
				}
			}

		}
		
		expected_alone_waiting_time = (*itr)->Enqueue_time + (*itr)->Estimated_alone_waiting_time	// T_last_TR_enqueued +T_last_TR_alone_waiting_time
			+ _NVMController->Estimate_transfer_and_memory_time(*itr);
		if (expected_alone_waiting_time > Simulator->Time())
			expected_alone_waiting_time -= Simulator->Time();
		else
			expected_alone_waiting_time = 0;

		(*position)->Estimated_alone_waiting_time = expected_alone_waiting_time;
		return (*position)->Estimated_alone_waiting_time;
	}

	// Added Function for using FLIN Algorithm. By Seong Jihwan.
	// This function estimate slowdown of flow of Transaction completely serviced at Backend of SSD.
	void TSU_FLIN::estimate_flow_slowdown(NVM_Transaction_Flash* serviced_TR)
	{

		NVM_Transaction_Flash* TR = serviced_TR;
		sim_time_type alone_turnaround = TR->Estimated_alone_waiting_time + TR->Transfer_memory_time;
		if (serviced_TR->Type == Transaction_Type::READ) {

			flow_activity_info[TR->Address.ChannelID][TR->Address.ChipID][TR->Priority_class][TR->Stream_id].Sum_read_slowdown = (double)(Simulator->Time() - TR->Enqueue_time) / alone_turnaround;
		}
		else
		{
			flow_activity_info[TR->Address.ChannelID][TR->Address.ChipID][TR->Priority_class][TR->Stream_id].Sum_write_slowdown = (double)(Simulator->Time() - TR->Issue_time) / alone_turnaround;
		}
	}

	// estimating Fairenss of Transactions in Queue 
	// parmeter "flow_with_max_average_slowdown" is input/output variable representing most slowdowned Flow
	double TSU_FLIN::fairness_based_on_average_slowdown(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id, unsigned int priority_class, bool is_read, stream_id_type& flow_with_max_average_slowdown)
	{
		double slowdown_max = 0, slowdown_min = 10000000000000000000;

		if (is_read)
		{
			for (unsigned int i = 0; i < stream_count_per_priority_class[priority_class]; i++)
			{
				double average_slowdown = flow_activity_info[channel_id][chip_id][priority_class][stream_ids_per_priority_class[priority_class][i]].Sum_RD_TR_shared_turnaround_time / flow_activity_info[channel_id][chip_id][priority_class][stream_ids_per_priority_class[priority_class][i]].Sum_RD_TR_alone_turnaround_time;
				if (average_slowdown > slowdown_max)
				{
					slowdown_max = average_slowdown;
					flow_with_max_average_slowdown = stream_ids_per_priority_class[priority_class][i];
				}
				if (average_slowdown < slowdown_min)
					slowdown_min = average_slowdown;
			}
		}
		else
		{
			for (unsigned int i = 0; i < stream_count_per_priority_class[priority_class]; i++)
			{
				double average_slowdown = flow_activity_info[channel_id][chip_id][priority_class][stream_ids_per_priority_class[priority_class][i]].Sum_WR_TR_shared_turnaround_time / flow_activity_info[channel_id][chip_id][priority_class][stream_ids_per_priority_class[priority_class][i]].Sum_WR_TR_alone_turnaround_time;
				if (average_slowdown > slowdown_max)
				{
					slowdown_max = average_slowdown;
					flow_with_max_average_slowdown = stream_ids_per_priority_class[priority_class][i];
				}
				if (average_slowdown < slowdown_min)
					slowdown_min = average_slowdown;
			}
		}

		return (double)slowdown_min / slowdown_max;
	}

	// moving inserted new Transaction to head of high intensity transacions in Queue ,as decribed in section 4.1
	void TSU_FLIN::move_forward(Flash_Transaction_Queue* queue, std::list<NVM_Transaction_Flash*>::iterator TRnew_pos, std::list<NVM_Transaction_Flash*>::iterator ultimate_posistion)
	{
		if (TRnew_pos == ultimate_posistion)
			return;
		auto Tnew_final_pos = TRnew_pos;
		Tnew_final_pos--;//TR_pos = TRnew_pos prev

		while ((*Tnew_final_pos)->Stream_id != (*TRnew_pos)->Stream_id && !(*Tnew_final_pos)->FLIN_Barrier && Tnew_final_pos != ultimate_posistion)
		{
			
			Tnew_final_pos--;
		}


		if (Tnew_final_pos == ultimate_posistion
			&& (*Tnew_final_pos)->Stream_id != (*ultimate_posistion)->Stream_id
			&& !(*TRnew_pos)->FLIN_Barrier)
		{
			NVM_Transaction_Flash* tr = *TRnew_pos;
			queue->remove(TRnew_pos);
			queue->insert(Tnew_final_pos, tr);
			ultimate_posistion = prev(Tnew_final_pos);
			tr->FLIN_Barrier = true;//According to FLIN: When TRnew is moved forward, it is tagged so that no future arriving flash transaction of the high-intensity flows jumps ahead of it
		}
		else
		{
			NVM_Transaction_Flash* tr = *TRnew_pos;
			queue->remove(TRnew_pos);
			queue->insert(Tnew_final_pos, tr);
			if (Tnew_final_pos==ultimate_posistion)
			{
				ultimate_posistion = prev(Tnew_final_pos);
			}
			tr->FLIN_Barrier = true;//According to FLIN: When TRnew is moved forward, it is tagged so that no future arriving flash transaction of the high-intensity flows jumps ahead of it
		}
	}

	// checking whether Queue[channel][chip][prioirty_ID] has transacion or not for scheduling Queue.
	bool TSU_FLIN::check_UserTRQueue(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id, bool isread)
	{
		bool TR_existed = false;
		if (isread)
		{

			for (unsigned int i = 0; i < no_of_priority_classes; i++)
			{

				if (UserReadTRQueue[channel_id][chip_id][i].size() > 0)
					TR_existed = true;

				
			}
			return TR_existed;
			
		}
		else {
			for (unsigned int i = 0; i < no_of_priority_classes; i++)
			{

				if (UserWriteTRQueue[channel_id][chip_id][i].size() > 0)
					TR_existed = true;


			}

			return TR_existed;
		}
	}

	void TSU_FLIN::initialize_scheduling_turns()
	{
	}

	void TSU_FLIN::update_head_high_TR(Flash_Transaction_Queue::iterator transaction)
	{
		auto itr = transaction;


		if ((*itr)->Type == Transaction_Type::READ)
		{
			if (head_high_read[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class]
				== UserReadTRQueue[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class].end())
				return;
			if (head_high_read[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] == itr)
				head_high_read[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] = std::next(itr);
		}
		else {
			if (head_high_write[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class]
				== UserWriteTRQueue[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class].end())
				return;
			if (head_high_write[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] == itr)
				head_high_write[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] = std::next(itr);
		}
	}
	

	// servicing read transacions by scheduling read transacion Queue from mapping read  Queue, GC read Queue and User read Queue 
	bool TSU_FLIN::service_read_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{
		bool isread = true;
		Flash_Transaction_Queue* sourceQueue1 = NULL, * sourceQueue2 = NULL;
		Flash_Transaction_Queue* userQueue = NULL;
		__int8 TRslotExisted = 0;
		bool FLIN_func = false;
		//Flash transactions that are related to FTL mapping data have the highest priority

		if (MappingReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
		{
			sourceQueue1 = &MappingReadTRQueue[chip->ChannelID][chip->ChipID];

			if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip) && GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue2 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
			}
			else
			{
				FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, isread);
				userQueue = UserReadTRQueue[chip->ChannelID][chip->ChipID];
				if (FLIN_func)
					TRslotExisted |= (1 << 1);

			}
		}
		else if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))
		{
			//If flash transactions related to GC are prioritzed (non-preemptive execution mode of GC), then GC queues are checked first
			if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];

				FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, isread);
				userQueue = UserReadTRQueue[chip->ChannelID][chip->ChipID];
				if (FLIN_func)
					TRslotExisted |= (1 << 1);

			}
			else if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				return false;
			}
			else if (GCEraseTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				return false;
			}
			else
			{
				FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, isread);
				userQueue = UserReadTRQueue[chip->ChannelID][chip->ChipID];

				if (!FLIN_func)
				{
					return false;
				}
				else {
					TRslotExisted |= (1 << 0);
				}
			}
		}
		else
		{
			//If GC is currently executed in the preemptive mode, then user IO transaction queues are checked first
			FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, isread);
			userQueue = UserReadTRQueue[chip->ChannelID][chip->ChipID];


			if (FLIN_func)
			{
				TRslotExisted |= (1 << 0);
				if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				{
					sourceQueue2 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
				}
			}
			else if (check_UserTRQueue(chip->ChannelID, chip->ChipID, false))
			{
				return false;
			}
			else if (GCReadTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &GCReadTRQueue[chip->ChannelID][chip->ChipID];
			}
			else
			{
				return false;
			}
		}

		bool suspensionRequired = false;
		ChipStatus cs = _NVMController->GetChipStatus(chip);
		switch (cs)
		{
		case ChipStatus::IDLE:
			break;
		case ChipStatus::WRITING:
			if (!programSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
			{
				return false;
			}
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < writeReasonableSuspensionTimeForRead)
			{
				return false;
			}
			suspensionRequired = true;
		case ChipStatus::ERASING:
			if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
			{
				return false;
			}
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForRead)
			{
				return false;
			}
			suspensionRequired = true;
		default:
			return false;
		}

		if (TRslotExisted == (1 << 0)) {
			issue_command_to_chip_FLIN(sourceQueue2, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else if (TRslotExisted == (1 << 1)) {
			issue_command_to_chip_FLIN(sourceQueue1, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else {
			issue_command_to_chip(sourceQueue1, sourceQueue2, Transaction_Type::READ, suspensionRequired);
		}
		return true;
	}

	// servicing write transacions by scheduling write transacion Queue from mapping write Queue, GC write Queue and User write Queue 
	bool TSU_FLIN::service_write_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{

		Flash_Transaction_Queue* sourceQueue1 = NULL, * sourceQueue2 = NULL;
		Flash_Transaction_Queue* userQueue = NULL;
		__int8 TRslotExisted = 0;
		bool FLIN_func = false;

		//If flash transactions related to GC are prioritzed (non-preemptive execution mode of GC), then GC queues are checked first
		if (ftl->GC_and_WL_Unit->GC_is_in_urgent_mode(chip))
		{
			if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];

				FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, false);
				userQueue = UserWriteTRQueue[chip->ChannelID][chip->ChipID];
				if (FLIN_func)
					TRslotExisted |= (1 << 1);

			}
			else if (GCEraseTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				return false;
			}
			else
			{ 
				FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, false);
				userQueue = UserWriteTRQueue[chip->ChannelID][chip->ChipID];

				if (!FLIN_func)
				{
					return false;
				}
				else {
					TRslotExisted |= (1 << 0);
				}
			}
		}
		else
		{
			//If GC is currently executed in the preemptive mode, then user IO transaction queues are checked first
			FLIN_func = check_UserTRQueue(chip->ChannelID, chip->ChipID, false);
			userQueue = UserWriteTRQueue[chip->ChannelID][chip->ChipID];


			if (FLIN_func)
			{
				TRslotExisted |= (1 << 0);
				if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
				{
					sourceQueue2 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
				}
			}
			else if (GCWriteTRQueue[chip->ChannelID][chip->ChipID].size() > 0)
			{
				sourceQueue1 = &GCWriteTRQueue[chip->ChannelID][chip->ChipID];
			}
			else
			{
				return false;
			}
		}

		bool suspensionRequired = false;
		ChipStatus cs = _NVMController->GetChipStatus(chip);
		switch (cs)
		{
		case ChipStatus::IDLE:
			break;
		case ChipStatus::ERASING:
			if (!eraseSuspensionEnabled || _NVMController->HasSuspendedCommand(chip))
				return false;
			if (_NVMController->Expected_finish_time(chip) - Simulator->Time() < eraseReasonableSuspensionTimeForWrite)
				return false;
			suspensionRequired = true;
		default:
			return false;
		}

		if (TRslotExisted == (1 << 0)) {
			issue_command_to_chip_FLIN(sourceQueue2, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else if (TRslotExisted == (1 << 1)) {
			issue_command_to_chip_FLIN(sourceQueue1, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else {
			issue_command_to_chip(sourceQueue1, sourceQueue2, Transaction_Type::WRITE, suspensionRequired);
		}


		return true;
	}

	bool TSU_FLIN::service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip)
	{  

		if (_NVMController->GetChipStatus(chip) != ChipStatus::IDLE)
		{
			return false;
		}

		Flash_Transaction_Queue* source_queue = &GCEraseTRQueue[chip->ChannelID][chip->ChipID];
		if (source_queue->size() == 0)
		{
			return false;
		}

		issue_command_to_chip(source_queue, NULL, Transaction_Type::ERASE, false);

		return true;
	}


	void TSU_FLIN::Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter)
	{
		name_prefix = name_prefix + ".TSU";
		xmlwriter.Write_open_tag(name_prefix);

		TSU_Base::Report_results_in_XML(name_prefix, xmlwriter);

		std::string attr = "ReOrdering_loop_count";
		std::string val = std::to_string(reordering_loop_count);
		xmlwriter.Write_attribute_string(attr, val);

		attr = "ReOrdering_success_loop_count";
		val = std::to_string(reordering_success_loop_count);
		xmlwriter.Write_attribute_string(attr, val);


		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				for (unsigned int priorityClass = 0; priorityClass < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priorityClass++)
				{
					UserReadTRQueue[channelID][chip_cntr][priorityClass].Report_results_in_XML(name_prefix + ".User_Read_TR_Queue.Priority." + IO_Flow_Priority_Class::to_string(priorityClass), xmlwriter);
				}
			}
		}

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				for (unsigned int priorityClass = 0; priorityClass < IO_Flow_Priority_Class::NUMBER_OF_PRIORITY_LEVELS; priorityClass++)
				{
					UserWriteTRQueue[channelID][chip_cntr][priorityClass].Report_results_in_XML(name_prefix + ".User_Write_TR_Queue.Priority." + IO_Flow_Priority_Class::to_string(priorityClass), xmlwriter);
				}
			}
		}

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				MappingReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".Mapping_Read_TR_Queue", xmlwriter);
			}
		}

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				MappingWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".Mapping_Write_TR_Queue", xmlwriter);
			}
		}

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				GCReadTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Read_TR_Queue", xmlwriter);
			}
		}

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				GCWriteTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Write_TR_Queue", xmlwriter);
			}
		}

		for (unsigned int channelID = 0; channelID < channel_count; channelID++)
		{
			for (unsigned int chip_cntr = 0; chip_cntr < chip_no_per_channel; chip_cntr++)
			{
				GCEraseTRQueue[channelID][chip_cntr].Report_results_in_XML(name_prefix + ".GC_Erase_TR_Queue", xmlwriter);
			}
		}

		xmlwriter.Write_close_tag();
	}



}


