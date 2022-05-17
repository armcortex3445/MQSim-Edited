#include "TSU_CIF.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include <stack>
#include <cmath>
#include "TSU_Priority_OutOfOrder.h"
#include "Host_Interface_Defs.h"
#include <assert.h>

namespace SSD_Components
{
	TSU_CIF::TSU_CIF(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController,
		const unsigned int channel_count, const unsigned int chip_no_per_channel, const unsigned int die_no_per_chip, const unsigned int plane_no_per_die, unsigned int flash_page_size,
		const unsigned int no_of_priority_classes, const stream_id_type max_flow_id, const unsigned int* stream_count_per_priority_class, stream_id_type** stream_ids_per_priority_class,
		const sim_time_type WriteReasonableSuspensionTimeForRead,
		const sim_time_type EraseReasonableSuspensionTimeForRead,
		const sim_time_type EraseReasonableSuspensionTimeForWrite,
		const bool EraseSuspensionEnabled, const bool ProgramSuspensionEnabled)
		:
		TSU_Base(id, ftl, NVMController, Flash_Scheduling_Type::CIF, channel_count, chip_no_per_channel, die_no_per_chip, plane_no_per_die,
			WriteReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForRead, EraseReasonableSuspensionTimeForWrite,
			EraseSuspensionEnabled, ProgramSuspensionEnabled),
		no_of_priority_classes(no_of_priority_classes)
	{
		this->stream_ids_per_priority_class = new stream_id_type * [no_of_priority_classes];
		this->stream_count_per_priority_class = new unsigned int[no_of_priority_classes];
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


		read_transaction_count = new int*** [channel_count];
		write_transaction_count = new int*** [channel_count];
		read_flow_intensity_bitmap = new uint8_t * *[channel_count];
		write_flow_intensity_bitmap = new uint8_t * *[channel_count];
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
			read_transaction_count[channel_id] = new int** [chip_no_per_channel];
			write_transaction_count[channel_id] = new int** [chip_no_per_channel];
			read_flow_intensity_bitmap[channel_id] = new uint8_t * [chip_no_per_channel];
			write_flow_intensity_bitmap[channel_id] = new uint8_t * [chip_no_per_channel];
			head_high_read[channel_id] = new Flash_Transaction_Queue::iterator * [chip_no_per_channel];
			head_high_write[channel_id] = new Flash_Transaction_Queue::iterator * [chip_no_per_channel];
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				read_transaction_count[channel_id][chip_id] = new int* [no_of_priority_classes];
				write_transaction_count[channel_id][chip_id] = new int* [no_of_priority_classes];
				read_flow_intensity_bitmap[channel_id][chip_id] = new uint8_t[no_of_priority_classes];
				write_flow_intensity_bitmap[channel_id][chip_id] = new uint8_t[no_of_priority_classes];
				head_high_read[channel_id][chip_id] = new Flash_Transaction_Queue::iterator[no_of_priority_classes];
				head_high_write[channel_id][chip_id] = new Flash_Transaction_Queue::iterator[no_of_priority_classes];



				UserReadTRQueue[channel_id][chip_id] = new Flash_Transaction_Queue[no_of_priority_classes];
				UserWriteTRQueue[channel_id][chip_id] = new Flash_Transaction_Queue[no_of_priority_classes];


				GCReadTRQueue[channel_id][chip_id].Set_id("GC_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@");
				MappingReadTRQueue[channel_id][chip_id].Set_id("Mapping_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				MappingWriteTRQueue[channel_id][chip_id].Set_id("Mapping_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCWriteTRQueue[channel_id][chip_id].Set_id("GC_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				GCEraseTRQueue[channel_id][chip_id].Set_id("GC_Erase_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id));
				for (unsigned int pclass_id = 0; pclass_id < no_of_priority_classes; pclass_id++)
				{
					read_transaction_count[channel_id][chip_id][pclass_id] = new int[max_flow_id];
					write_transaction_count[channel_id][chip_id][pclass_id] = new int[max_flow_id];
					for (int i = 0; i < max_flow_id; i++) {
						read_transaction_count[channel_id][chip_id][pclass_id][i] = 0;
						write_transaction_count[channel_id][chip_id][pclass_id][i] = 0;
					}


					UserReadTRQueue[channel_id][chip_id][pclass_id].Set_id("User_Read_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@" + std::to_string(pclass_id));
					UserWriteTRQueue[channel_id][chip_id][pclass_id].Set_id("User_Write_TR_Queue@" + std::to_string(channel_id) + "@" + std::to_string(chip_id) + "@" + std::to_string(pclass_id));
					head_high_read[channel_id][chip_id][pclass_id] = UserReadTRQueue[channel_id][chip_id][pclass_id].end();
					head_high_write[channel_id][chip_id][pclass_id] = UserWriteTRQueue[channel_id][chip_id][pclass_id].end();
				}
			}
		}
	}

	TSU_CIF::~TSU_CIF()
	{

		for (unsigned int channel_id = 0; channel_id < channel_count; channel_id++)
		{
			for (unsigned int chip_id = 0; chip_id < chip_no_per_channel; chip_id++)
			{
				delete[] UserReadTRQueue[channel_id][chip_id];
				delete[] UserWriteTRQueue[channel_id][chip_id];


				for (unsigned int pclass_id = 0; pclass_id < no_of_priority_classes; pclass_id++)
				{
					delete[] read_transaction_count[channel_id][chip_id][pclass_id];
					delete[] write_transaction_count[channel_id][chip_id][pclass_id];
				}


				delete[] head_high_read[channel_id][chip_id];
				delete[] head_high_write[channel_id][chip_id];
				delete[] read_transaction_count[channel_id][chip_id];
				delete[] write_transaction_count[channel_id][chip_id];
				delete[] read_flow_intensity_bitmap[channel_id][chip_id];
				delete[] write_flow_intensity_bitmap[channel_id][chip_id];
			}
			delete[] UserReadTRQueue[channel_id];
			delete[] UserWriteTRQueue[channel_id];
			delete[] GCReadTRQueue[channel_id];
			delete[] GCWriteTRQueue[channel_id];
			delete[] GCEraseTRQueue[channel_id];
			delete[] MappingReadTRQueue[channel_id];
			delete[] MappingWriteTRQueue[channel_id];

			delete[] head_high_read[channel_id];
			delete[] head_high_write[channel_id];
			delete[] read_transaction_count[channel_id];
			delete[] write_transaction_count[channel_id];
			delete[] read_flow_intensity_bitmap[channel_id];
			delete[] write_flow_intensity_bitmap[channel_id];

		}
		delete[] UserReadTRQueue;
		delete[] UserWriteTRQueue;
		delete[] GCReadTRQueue;
		delete[] GCWriteTRQueue;
		delete[] GCEraseTRQueue;
		delete[] MappingReadTRQueue;
		delete[] MappingWriteTRQueue;


		delete[] head_high_read;
		delete[] head_high_write;
		//내가 추가
		delete[] read_transaction_count;
		delete[] write_transaction_count;
		delete[] read_flow_intensity_bitmap;
		delete[] write_flow_intensity_bitmap;
	}
	void TSU_CIF::Start_simulation() {}


	void TSU_CIF::Validate_simulation_config() {}


	void TSU_CIF::Execute_simulator_event(MQSimEngine::Sim_Event* event) {}

	void TSU_CIF::Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter)
	{
		name_prefix = name_prefix + ".TSU";
		xmlwriter.Write_open_tag(name_prefix);

		TSU_Base::Report_results_in_XML(name_prefix, xmlwriter);
		std::string attr = "compensate_loop_count";
		std::string val = std::to_string(compensate_loop_count);
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



	inline void TSU_CIF::Prepare_for_transaction_submit()
	{
		opened_scheduling_reqs++;
		if (opened_scheduling_reqs > 1)
			return;

		transaction_receive_slots.clear();
	}

	inline void TSU_CIF::Submit_transaction(NVM_Transaction_Flash* transaction)
	{
		transaction_receive_slots.push_back(transaction);
	}
	// Scheduling transaction by using CIF Algorithm. 
	void TSU_CIF::Schedule()
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

			stream_id_type flow_id = (*it)->Stream_id;


			switch ((*it)->Type)
			{

			case Transaction_Type::READ:
				switch ((*it)->Source)
				{

				case Transaction_Source_Type::CACHE:
				case Transaction_Source_Type::USERIO:

					if (stream_count_per_priority_class[priority_class] < 2)
					{
						UserReadTRQueue[channel_id][chip_id][priority_class].push_back(*it);
						break;
					}


					read_transaction_count[channel_id][chip_id][priority_class][flow_id]++;
					if (read_transaction_count[channel_id][chip_id][priority_class][flow_id] > get_avg_TR_count(priority_class, read_transaction_count[channel_id][chip_id][priority_class]))
					{
						read_flow_intensity_bitmap[channel_id][chip_id][priority_class] |= (1 << flow_id);
						auto position = find_position_for_compensate(&UserReadTRQueue[channel_id][chip_id][priority_class], *it, read_transaction_count[channel_id][chip_id][priority_class], read_flow_intensity_bitmap[channel_id][chip_id][priority_class]);

						UserReadTRQueue[channel_id][chip_id][priority_class].insert(position, *it);
						if (position == head_high_read[channel_id][chip_id][priority_class])
							head_high_read[channel_id][chip_id][priority_class] = std::prev(position);
					}
					else
					{
						read_flow_intensity_bitmap[channel_id][chip_id][priority_class] &= ~(1 << flow_id);
						auto position = find_position_for_compensate(&UserReadTRQueue[channel_id][chip_id][priority_class], *it, read_transaction_count[channel_id][chip_id][priority_class], read_flow_intensity_bitmap[channel_id][chip_id][priority_class]);
						UserReadTRQueue[channel_id][chip_id][priority_class].insert(position, *it);
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



					if (stream_count_per_priority_class[priority_class] < 2)
					{
						UserWriteTRQueue[channel_id][chip_id][priority_class].push_back(*it);
						break;
					}


					write_transaction_count[channel_id][chip_id][priority_class][flow_id]++;
					if (write_transaction_count[channel_id][chip_id][priority_class][flow_id] > get_avg_TR_count(priority_class, write_transaction_count[channel_id][chip_id][priority_class]))
					{
						write_flow_intensity_bitmap[channel_id][chip_id][priority_class] |= (1 << flow_id);
						auto position = find_position_for_compensate(&UserWriteTRQueue[channel_id][chip_id][priority_class], *it, write_transaction_count[channel_id][chip_id][priority_class], write_flow_intensity_bitmap[channel_id][chip_id][priority_class]);
						UserWriteTRQueue[channel_id][chip_id][priority_class].insert(position, *it);
						if (position == head_high_write[channel_id][chip_id][priority_class])
							head_high_write[channel_id][chip_id][priority_class] = std::prev(position);
					}
					else
					{
						write_flow_intensity_bitmap[channel_id][chip_id][priority_class] &= ~(1 << flow_id);
						auto position = find_position_for_compensate(&UserWriteTRQueue[channel_id][chip_id][priority_class], *it, write_transaction_count[channel_id][chip_id][priority_class], write_flow_intensity_bitmap[channel_id][chip_id][priority_class]);
						UserWriteTRQueue[channel_id][chip_id][priority_class].insert(position, *it);
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

	//updating head of Highintensity intensity 
	void TSU_CIF::update_head_high_TR(Flash_Transaction_Queue::iterator transaction)
	{
		auto itr = transaction;


		if ((*itr)->Type == Transaction_Type::READ)
		{
			if (head_high_read[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class]
				== UserReadTRQueue[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class].end()) {
				return;
			}
			else if (head_high_read[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] == itr) {
				head_high_read[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] = std::next(itr);
			}
		}
		else {
			if (head_high_write[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class]
				== UserWriteTRQueue[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class].end())
				return;
			if (head_high_write[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] == itr)
				head_high_write[(*itr)->Address.ChannelID][(*itr)->Address.ChipID][(*itr)->Priority_class] = std::next(itr);
		}
	}

	bool TSU_CIF::service_read_transaction(NVM::FlashMemory::Flash_Chip* chip)
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
			issue_command_to_chip_CIF(sourceQueue2, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else if (TRslotExisted == (1 << 1)) {
			issue_command_to_chip_CIF(sourceQueue1, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else {
			issue_command_to_chip(sourceQueue1, sourceQueue2, Transaction_Type::READ, suspensionRequired);
		}
		return true;
	}

	bool TSU_CIF::service_write_transaction(NVM::FlashMemory::Flash_Chip* chip)
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
			issue_command_to_chip_CIF(sourceQueue2, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else if (TRslotExisted == (1 << 1)) {
			issue_command_to_chip_CIF(sourceQueue1, userQueue, no_of_priority_classes, suspensionRequired, TRslotExisted);
		}
		else {
			issue_command_to_chip(sourceQueue1, sourceQueue2, Transaction_Type::WRITE, suspensionRequired);
		}


		return true;
	}

	bool TSU_CIF::service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip)
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



	//This function get average transacion count of flows in read/write Queue[channel_ID][Chip_ID][Priority] 
	inline int TSU_CIF::get_avg_TR_count(int priority_class, int* transaction_count)
	{
		int sum = 0;
		for (int i = 0; i < stream_count_per_priority_class[priority_class]; i++)
			sum += transaction_count[stream_ids_per_priority_class[priority_class][i]];

		return sum / (int)stream_count_per_priority_class[priority_class];
	}

	//finding position maximizing fairness of Queue for new transaction.
	Flash_Transaction_Queue::iterator TSU_CIF::find_position_for_compensate(Flash_Transaction_Queue* queue, NVM_Transaction_Flash* TRnew, int* transaction_count, uint8_t flow_intensity_bitmap)
	{
		// In Qeueue, Transactions are classified and ordered by IO Intensity.

		//			  <-------Low intensity Transaction ------------>|<---- High intensity Transaction -------->  			
		//   Queue = [begin] [begin + 1] ...[low_tail -1 ] [low_tail]|[head_High ] [Head_High + 1 ] .... [end-1] [NULL]
		//			   head  <---------------------------------------------------------------------------------> tail
		//			 begin() <---------------------------------------------------------------------------------> end()
		//															<== travel direction for finding position

		int avg_Intensity_others = 0;
		int compensate_count = 0;
		int count = 0;
		int priority_class = TRnew->Priority_class;
		bool IsHigh;
		std::list<NVM_Transaction_Flash*>::iterator  position;

		//Step 0 : claculating compensate_count of New transaction, by using average transaction count in Queue.
		if (flow_intensity_bitmap & (1 << TRnew->Stream_id)) {
			//if new Transaction is High intensity, average Io intensity is calculated excluding transaction count of low intensity.
			IsHigh = true;
			for (int i = 0; i < stream_count_per_priority_class[priority_class]; i++) {
				if (flow_intensity_bitmap & (1 << stream_ids_per_priority_class[priority_class][i])) {
					avg_Intensity_others += transaction_count[stream_ids_per_priority_class[priority_class][i]];
					count++;
				}
			}
			// position start from tail of Queue
			position = queue->end();
			if (count == 1) {
				return position;

			}
			avg_Intensity_others -= transaction_count[TRnew->Stream_id];
			count--;
			avg_Intensity_others = avg_Intensity_others / count;
			compensate_count = avg_Intensity_others - transaction_count[TRnew->Stream_id];


		}
		else {
			//if new Transaction is Low intensity, average Io intensity is calculated excluding transaction count of High intensity.
			IsHigh = false;
			for (int i = 0; i < stream_count_per_priority_class[priority_class]; i++) {
				if (!(flow_intensity_bitmap & (1 << stream_ids_per_priority_class[priority_class][i]))) {
					avg_Intensity_others += transaction_count[stream_ids_per_priority_class[priority_class][i]];
					count++;
				}
			}
			// position start from head_high 
			if (TRnew->Type == Transaction_Type::READ) {
				position = head_high_read[TRnew->Address.ChannelID][TRnew->Address.ChipID][priority_class];
			}
			else {
				position = head_high_write[TRnew->Address.ChannelID][TRnew->Address.ChipID][priority_class];
			}
			if (count == 1) {
				return position;

			}
			avg_Intensity_others -= transaction_count[TRnew->Stream_id];
			count--;
			avg_Intensity_others = avg_Intensity_others / count;
			compensate_count = avg_Intensity_others - transaction_count[TRnew->Stream_id];

		}

		//Step 1 : find position for new transacion to move, by traveling.
		for (int i = 0; i < compensate_count; i++) {
			compensate_loop_count++;
			


			if (position != queue->end()) {
				// compensation is break, when next situations happen
				// 1) transaction located at position is from flow of new transaction. => to prevent interference between transaction from same flow.
				// 2) transaction located at position if from lower intensity flow than new transaction => to prevent IO intensity interference.
				// 3) for Exception handling status.
				if (((*position)->Stream_id == TRnew->Stream_id)
					|| transaction_count[(*position)->Stream_id] < transaction_count[TRnew->Stream_id]
					|| position == queue->begin()) {
					break;
				}
			}

			position--;
		}


		return position;

	}

	void TSU_CIF::update_transaction_count(NVM_Transaction_Flash* transaction)
	{
		auto TR = transaction;
		if (TR->Type == Transaction_Type::READ)
		{
			read_transaction_count[TR->Address.ChannelID][TR->Address.ChipID][TR->Priority_class][TR->Stream_id]--;
		}
		else
		{
			write_transaction_count[TR->Address.ChannelID][TR->Address.ChipID][TR->Priority_class][TR->Stream_id]--;
		}
	}

	bool TSU_CIF::check_UserTRQueue(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id, bool isread)
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

}