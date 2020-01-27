/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : ble_list.c
* Author             : AMS - HEA&RF BU
* Version            : V1.0.0
* Date               : 19-July-2012
* Description        : Circular Linked List Implementation.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/******************************************************************************
 * Include Files
******************************************************************************/
#include <hal_types.h>
#include "ble_list.h"
#include "stm32f4xx_hal.h"


/******************************************************************************
 * Function Definitions 
******************************************************************************/
void list_init_head (tListNode * listHead)
{
  listHead->next = listHead;
  listHead->prev = listHead;	
}

uint8_t list_is_empty (tListNode * listHead)
{
  uint32_t uwPRIMASK_Bit;
  uint8_t return_value;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  if(listHead->next == listHead)
  {
    return_value = TRUE;
  }
  else
  {
    return_value = FALSE;
  }
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/

  return return_value;
}

void list_insert_head (tListNode * listHead, tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  node->next = listHead->next;
  node->prev = listHead;
  listHead->next = node;
  (node->next)->prev = node;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_insert_tail (tListNode * listHead, tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  node->next = listHead;
  node->prev = listHead->prev;
  listHead->prev = node;
  (node->prev)->next = node;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_remove_node (tListNode * node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  (node->prev)->next = node->next;
  (node->next)->prev = node->prev;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_remove_head (tListNode * listHead, tListNode ** node )
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  *node = listHead->next;
  list_remove_node (listHead->next);
  (*node)->next = NULL;
  (*node)->prev = NULL;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_remove_tail (tListNode * listHead, tListNode ** node )
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  *node = listHead->prev;
  list_remove_node (listHead->prev);
  (*node)->next = NULL;
  (*node)->prev = NULL;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_insert_node_after (tListNode * node, tListNode * ref_node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  node->next = ref_node->next;
  node->prev = ref_node;
  ref_node->next = node;
  (node->next)->prev = node;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_insert_node_before (tListNode * node, tListNode * ref_node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  node->next = ref_node;
  node->prev = ref_node->prev;
  ref_node->prev = node;
  (node->prev)->next = node;
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

int list_get_size (tListNode * listHead)
{
  int size = 0;
  tListNode * temp;
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/

  temp = listHead->next;
  while (temp != listHead)
  {
    size++;
    temp = temp->next;		
  }
  
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
  return (size);
}

void list_get_next_node (tListNode * ref_node, tListNode ** node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  *node = ref_node->next;
    
  __set_PRIMASK(uwPRIMASK_Bit);     /**< Restore PRIMASK bit*/
}

void list_get_prev_node (tListNode * ref_node, tListNode ** node)
{
  uint32_t uwPRIMASK_Bit;

  uwPRIMASK_Bit = __get_PRIMASK();  /**< backup PRIMASK bit */
  __disable_irq();                  /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  
  *node = ref_node->prev;
  
  __set_PRIMASK(uwPRIMASK_Bit);      /**< Restore PRIMASK bit*/
}

