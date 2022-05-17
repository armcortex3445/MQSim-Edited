#include "NVM_PHY_ONFI.h"

namespace SSD_Components {
	// �Ű����� fuction�� vector �ڷᱸ���� �ִ´�.
	void NVM_PHY_ONFI::ConnectToTransactionServicedSignal(TransactionServicedHandlerType function)
	{
		
		connectedTransactionServicedHandlers.push_back(function);//TR�� ó���Ǿ��ٴ� ��ȣ��  GC_Wl unit���� �༭ flash resource�� �����ϵ��� �Ѵ�. 
	}

	/*
	* Different FTL components maybe waiting for a transaction to be finished:
	* HostInterface: For user reads and writes
	* Address_Mapping_Unit: For mapping reads and writes
	* TSU: For the reads that must be finished for partial writes (first read non updated parts of page data and then merge and write them into the new page)
	* GarbageCollector: For gc reads, writes, and erases
	*/
	void NVM_PHY_ONFI::broadcastTransactionServicedSignal(NVM_Transaction_Flash* transaction)
	{
		if (transaction->Type == Transaction_Type::WRITE && transaction->PPA == 52953343)
		{
			if (transaction->Source == Transaction_Source_Type::GC_WL)
			{
				std::cout << "GC WR TR->PPA == 52953343 is serviced . stream, lpa =  " << (transaction)->Stream_id << " ," << (transaction)->LPA << std::endl;

			}
			else
			{
				std::cout << "USER/CACHE WR TR->PPA == 52953343 is serviced . stream, lpa =  " << (transaction)->Stream_id << " ," << (transaction)->LPA << std::endl;
			}
		}
		for (std::vector<TransactionServicedHandlerType>::iterator it = connectedTransactionServicedHandlers.begin();
			it != connectedTransactionServicedHandlers.end(); it++) {
			(*it)(transaction);
		}
		delete transaction;//This transaction has been consumed and no more needed
	}

	void NVM_PHY_ONFI::ConnectToChannelIdleSignal(ChannelIdleHandlerType function)
	{
		connectedChannelIdleHandlers.push_back(function);
	}
	//TSU�� Channel�� IDLE�̶�� ��ȣ�� ������.
	void NVM_PHY_ONFI::broadcastChannelIdleSignal(flash_channel_ID_type channelID)
	{
		for (std::vector<ChannelIdleHandlerType>::iterator it = connectedChannelIdleHandlers.begin();
			it != connectedChannelIdleHandlers.end(); it++) {
			(*it)(channelID);
		}
	}

	void NVM_PHY_ONFI::ConnectToChipIdleSignal(ChipIdleHandlerType function)
	{
		connectedChipIdleHandlers.push_back(function);
	}
	//TSU�� Chip�� IDLE�̶�� ��ȣ�� �ش�.
	void NVM_PHY_ONFI::broadcastChipIdleSignal(NVM::FlashMemory::Flash_Chip* chip)
	{
		for (std::vector<ChipIdleHandlerType>::iterator it = connectedChipIdleHandlers.begin();
			it != connectedChipIdleHandlers.end(); it++) {
			(*it)(chip);
		}
	}
}