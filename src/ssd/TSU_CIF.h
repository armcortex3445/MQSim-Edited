#ifndef TSU_CIF_H
#define TSU_CIF_H

#include <list>
#include <set>
#include "TSU_Base.h"
#include "NVM_Transaction_Flash.h"
#include "NVM_PHY_ONFI_NVDDR2.h"
#include "FTL.h"


namespace SSD_Components
{
	class FTL;


	/* TSU_CIF class is made to implement Compensate I/O Intensity for Fairness Algorithm By Seong jihwan.
	* CIF Algorithm is my capstone Designe at SeoulTech university.
	* To make TUS_CIF class, I reference a FLIN Algorithm and MQ Sim source code.
	*
	* - Overview of CIF Algorithm.
	* CIF Algorithm purposes to reduce overhead of reordering Transaction in FLIN Algorithm.
	* CIF Algorithm just uses count of transacion from flow in Queue. By using count, reordering Tranaction occurs.
	* High intensity flow will insert a lot of transactions in Queue. on the other hands,Low intensity flow will insert a few of ones.
	* for fairness imprvoement, CIF Algorithm moves transacion from flow inserting a few of transactions to head.
	* moving steps is calculated by difference between average count of transaction of flow in Queue and count of transaction of low intensity flow in Queue.
	* As a result, CIF Algorithm compensates IO intensity by moving transaction to head of Queue.
	*
	*
	* - reference : To design CIF Algorithm, I refered these.
	* [1] ¡°MQSim GitHub Repository,¡± https://github.com/CMU-SAFARI/MQSim.
	* [2] A. Tavakkol et al., ¡°MQSim: A Framework for Enabling Realistic Studies of Modern Multi-Queue SSD Devices¡±, USENIX FAST, pp. 49 - 66, 2018.
	* [3] A. Tavakkol et al , ¡°:FLIN: Enabling Fairness and Enhancing Performance in Modern NVMeSolid State Drives ¡±, in IEEE 45thAnnual ISCA,pp.397-410 2018
	*/

	class TSU_CIF
		: public TSU_Base
	{
	public:
		TSU_CIF(const sim_object_id_type& id, FTL* ftl, NVM_PHY_ONFI_NVDDR2* NVMController,
			const unsigned int channel_count, const unsigned int chip_no_per_channel, const unsigned int die_no_per_chip, const unsigned int plane_no_per_die, unsigned int flash_page_size,
			const unsigned int no_of_priority_classes, const stream_id_type max_flow_id, const unsigned int* stream_count_per_priority_class, stream_id_type** stream_ids_per_priority_class,
			const sim_time_type WriteReasonableSuspensionTimeForRead,
			const sim_time_type EraseReasonableSuspensionTimeForRead,
			const sim_time_type EraseReasonableSuspensionTimeForWrite,
			const bool EraseSuspensionEnabled, const bool ProgramSuspensionEnabled);
		~TSU_CIF();
		void Prepare_for_transaction_submit();
		void Submit_transaction(NVM_Transaction_Flash* transaction);
		void Schedule();

		void Start_simulation();
		void Validate_simulation_config();
		void Execute_simulator_event(MQSimEngine::Sim_Event*);
		void Report_results_in_XML(std::string name_prefix, Utils::XmlWriter& xmlwriter);
	private:


		unsigned int no_of_priority_classes;
		// Queue[i][j[k] is meaning Transaction Queue in priority k of chip j of channel i.
		Flash_Transaction_Queue*** UserReadTRQueue;
		Flash_Transaction_Queue*** UserWriteTRQueue;
		Flash_Transaction_Queue** GCReadTRQueue;
		Flash_Transaction_Queue** GCWriteTRQueue;
		Flash_Transaction_Queue** GCEraseTRQueue;
		Flash_Transaction_Queue** MappingReadTRQueue;
		Flash_Transaction_Queue** MappingWriteTRQueue;


		stream_id_type** stream_ids_per_priority_class;
		unsigned int* stream_count_per_priority_class;
		int**** read_transaction_count;
		int**** write_transaction_count;
		uint8_t*** read_flow_intensity_bitmap;				// recording whether flow is High intensity or not. If there are 4 flows, read_bitmap[0][0][1] = 0b0000_0001 means Flow 0 is High intensity in ewS Queue located in priority 1 of chip 0 of channel 0.
		uint8_t*** write_flow_intensity_bitmap;				// recording whether flow is High intensity or not
		Flash_Transaction_Queue::iterator*** head_high_read;//Due to programming limitations, for read queues, MQSim keeps Head_high instread of Tail_low which is described in Alg 1 of the FLIN paper
		Flash_Transaction_Queue::iterator*** head_high_write;//Due to programming limitations, for write queues, MQSim keeps Head_high instread of Tail_low which is described in Alg 1 of the FLIN paper
		uint64_t compensate_loop_count = 0;// In find_position_for_compensate() , count of loop to move to position. This variable is used for esimating overhead of CIF Algorithm. 



		bool service_read_transaction(NVM::FlashMemory::Flash_Chip* chip);
		bool service_write_transaction(NVM::FlashMemory::Flash_Chip* chip);
		bool service_erase_transaction(NVM::FlashMemory::Flash_Chip* chip);

		bool check_UserTRQueue(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id, bool isread);

		// definition of function is described in TSU_CIF.cpp
		Flash_Transaction_Queue::iterator find_position_for_compensate(Flash_Transaction_Queue* queue, NVM_Transaction_Flash* TRnew, int* transaction_count, uint8_t flow_intensity_bitmap);
		void update_head_high_TR(Flash_Transaction_Queue::iterator transaction);
		int get_avg_TR_count(int priority_class, int* transaction_count);
		void update_transaction_count(NVM_Transaction_Flash* transaction);

	};
}


#endif //!TSU_CIF