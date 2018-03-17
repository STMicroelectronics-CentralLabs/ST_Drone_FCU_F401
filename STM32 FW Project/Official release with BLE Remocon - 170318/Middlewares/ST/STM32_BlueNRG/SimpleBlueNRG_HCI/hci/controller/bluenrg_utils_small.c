
#if (defined STM32_NUCLEO) || (defined STM32_BLUECOIN)
#include "hal.h"
#include "hal_types.h"
#include "ble_status.h"
#include "bluenrg_aci.h"
#include "bluenrg_utils.h"
#include "hci.h"
#include "osal.h"
#include "string.h"

#ifdef STM32_NUCLEO
  #include "stm32_bluenrg_ble.h"
#elif STM32_BLUECOIN
  #include "BlueCoin_BlueNRG.h"
#endif /* STM32_NUCLEO */


uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_le_read_local_version(&hci_version, &hci_revision, &lmp_pal_version, 
				     &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
    *fwVersion = (hci_revision & 0xFF) << 8;              // Major Version Number
    *fwVersion |= ((lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    *fwVersion |= lmp_pal_subversion & 0xF;               // Patch Version Number
  }

  HCI_Process(); // To receive the BlueNRG EVT

  return status;
}
#endif /* STM32_NUCLEO */
