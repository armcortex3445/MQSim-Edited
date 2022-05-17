#ifndef NVM_PHY_ONFI_NVDDR2_H
#define NVM_PHY_ONFI_NVDDR2_H

#include <queue>
#include <list>
#include "../sim/Sim_Defs.h"
#include "../nvm_chip/flash_memory/FlashTypes.h"
#include "../nvm_chip/flash_memory/Flash_Command.h"
#include "NVM_PHY_ONFI.h"
#include "ONFI_Channel_NVDDR2.h"
#include "Flash_Transaction_Queue.h"

namespace SSD_Components
{
	enum class NVDDR2_SimEventType
	{
		READ_DATA_TRANSFERRED, READ_CMD_ADDR_TRANSFERRED,
		PROGRAM_CMD_ADDR_DATA_TRANSFERRED, 
		PROGRAM_COPYBACK_CMD_ADDR_TRANSFERRED, 
		ERASE_SETUP_COMPLETED
	};
	//Die resource를 상태를 나타내며, Die resource를 관리하는데 사용된다.
	class DieBookKeepingEntry
	{
	public:
		NVM::FlashMemory::Flash_Command* ActiveCommand; //The current command that is executing on the die

		/*The current transactions that are being serviced. For the set of transactions in ActiveTransactions,
		there is one ActiveCommand that is geting executed on the die. Transaction is a FTL-level concept, and
		command is a flash chip-level concept*/
		/* 처리되고 있는 현재 TR. ActiveTransactions에서 TR의 집합때문에, die에서 실행되는 Activecommand는 1개이다.
		* TR은 FTL-level 개념이고, command는 flash chip-level 개념이다.
		*/
		//?? Command를 잘게 나누어서 TR을 만드는 건가?  1개 command는 복수의 TR로 구성?
		std::list<NVM_Transaction_Flash*> ActiveTransactions; // TSU에서 Target Die로 dispatch한 TR들. target plane이 서로 달라서 multi-plnae동작 여부를 파악할때 사용한다. 
		NVM::FlashMemory::Flash_Command* SuspendedCommand;
		std::list<NVM_Transaction_Flash*> SuspendedTransactions;
		NVM_Transaction_Flash* ActiveTransfer; //The current transaction . chip에서 read한 data를 저장한 TR을 나타냄. 해당 TR은 chip 밖으로 전송된다.
		bool Free;
		bool Suspended;
		sim_time_type Expected_finish_time;
		sim_time_type RemainingExecTime;

		//target Die에서 CMD transfer가 끝나는데 걸리는 시간.
		sim_time_type DieInterleavedTime;//If the command transfer is done in die-interleaved mode, the transfer time is recorded in this temporary variable


		//die를 suspension 시킬 준비를 한다. 상태를 바꾸고, active TR과 CMD를 Suspended에 저장
		void PrepareSuspend()
		{
			SuspendedCommand = ActiveCommand;
			RemainingExecTime = Expected_finish_time - Simulator->Time();
			SuspendedTransactions.insert(SuspendedTransactions.begin(), ActiveTransactions.begin(), ActiveTransactions.end());//Queue head에 Active Queue에 있는 모든 TR을 삽입
			Suspended = true;
			ActiveCommand = NULL;
			ActiveTransactions.clear();
			Free = true;
		}
		//suspension을 resume하기 위해 준비를한다.
		void PrepareResume()
		{
			ActiveCommand = SuspendedCommand;
			Expected_finish_time = Simulator->Time() + RemainingExecTime;
			ActiveTransactions.insert(ActiveTransactions.begin(), SuspendedTransactions.begin(), SuspendedTransactions.end());//Suspension되었던 모든 TR을 activeTR에 삽입
			Suspended = false;
			SuspendedCommand = NULL;
			SuspendedTransactions.clear();
			Free = false;
		}

		void ClearCommand()
		{
			delete ActiveCommand;
			ActiveCommand = NULL;
			ActiveTransactions.clear();
			Free = true;
		}
	};
	//Chip resource를 상태를 나타내며, Chip resource를 관리하는데 사용된다.
	class ChipBookKeepingEntry
	{
	public:
		ChipStatus Status;	//Chip의 상태를 나타낸다. IDLE,, CMD_IN, CMD_DATA_IN, DATA_OUT,READING 등.
		DieBookKeepingEntry* Die_book_keeping_records;		//Die -level에서 처리되는 TR에 관한 정보를 가지고있다.
		sim_time_type Expected_command_exec_finish_time;	//예상되는 command 종료시각
		sim_time_type Last_transfer_finish_time;
		bool HasSuspend;
		std::queue<DieBookKeepingEntry*> OngoingDieCMDTransfers;// 현재진행중이거나 곧 진행될 transfer CMD들의 target Die를 저장함. 앞에있는 Die먼저 transfer CMD를 실행
		unsigned int WaitingReadTXCount;//Chip에 있는 Die에서 Read한 data를 Transfer out해야하는 횟수, WaitingReadTX,WaitingGCRead_TX,WaitingMappingRead_TX에 있는 TR들의 합을 나타냄
		unsigned int No_of_active_dies;

		void PrepareSuspend() { HasSuspend = true; No_of_active_dies = 0; }//Chip이 Suspension됨, 동작중인 die = 0이 됨
		void PrepareResume() { HasSuspend = false; }//Chip이 Suspension이 종료됨
	};

	class NVM_PHY_ONFI_NVDDR2 : public NVM_PHY_ONFI
	{
	public:
		NVM_PHY_ONFI_NVDDR2(const sim_object_id_type& id, ONFI_Channel_NVDDR2** channels,
			unsigned int ChannelCount, unsigned int chip_no_per_channel, unsigned int DieNoPerChip, unsigned int PlaneNoPerDie);
		void Setup_triggers();
		void Validate_simulation_config();
		void Start_simulation();

		void Send_command_to_chip(std::list<NVM_Transaction_Flash*>& transactionList);
		void Change_flash_page_status_for_preconditioning(const NVM::FlashMemory::Physical_Page_Address& page_address, const LPA_type lpa);
		void Execute_simulator_event(MQSimEngine::Sim_Event*);
		BusChannelStatus Get_channel_status(flash_channel_ID_type channelID);
		NVM::FlashMemory::Flash_Chip* Get_chip(flash_channel_ID_type channel_id, flash_chip_ID_type chip_id);
		LPA_type Get_metadata(flash_channel_ID_type channe_id, flash_chip_ID_type chip_id, flash_die_ID_type die_id, flash_plane_ID_type plane_id, flash_block_ID_type block_id, flash_page_ID_type page_id);//A simplification to decrease the complexity of GC execution! The GC unit may need to know the metadata of a page to decide if a page is valid or invalid. 
		
																																																			
		bool HasSuspendedCommand(NVM::FlashMemory::Flash_Chip* chip);
		ChipStatus GetChipStatus(NVM::FlashMemory::Flash_Chip* chip);
		sim_time_type Expected_finish_time(NVM::FlashMemory::Flash_Chip* chip);
		sim_time_type Expected_finish_time(NVM_Transaction_Flash* transaction);
		sim_time_type Expected_transfer_time(NVM_Transaction_Flash* transaction);
		NVM_Transaction_Flash* Is_chip_busy_with_stream(NVM_Transaction_Flash* transaction);
		bool Is_chip_busy(NVM_Transaction_Flash* transaction);
		void Change_memory_status_preconditioning(const NVM::NVM_Memory_Address* address, const void* status_info);

		sim_time_type Estimate_transfer_and_memory_time(NVM_Transaction_Flash* transaction);//Added function for using FLIN Algorithm. By Seong Jihwan.

	private:
		void transfer_read_data_from_chip(ChipBookKeepingEntry* chipBKE, DieBookKeepingEntry* dieBKE, NVM_Transaction_Flash* tr);
		void perform_interleaved_cmd_data_transfer(NVM::FlashMemory::Flash_Chip* chip, DieBookKeepingEntry* bookKeepingEntry);
		void send_resume_command_to_chip(NVM::FlashMemory::Flash_Chip* chip, ChipBookKeepingEntry* chipBKE);
		static void handle_ready_signal_from_chip(NVM::FlashMemory::Flash_Chip* chip, NVM::FlashMemory::Flash_Command* command);

		static NVM_PHY_ONFI_NVDDR2* _my_instance;
		ONFI_Channel_NVDDR2** channels;
		ChipBookKeepingEntry** bookKeepingTable;	
		Flash_Transaction_Queue *WaitingReadTX, *WaitingGCRead_TX, *WaitingMappingRead_TX;
		std::list<DieBookKeepingEntry*> *WaitingCopybackWrites; 
	};
}

#endif // !NVM_PHY_ONFI_NVDDR2_H
