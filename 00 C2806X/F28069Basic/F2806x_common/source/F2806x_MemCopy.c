// TI File $Revision: /main/2 $
// Checkin $Date: January 4, 2011   10:10:32 $
//###########################################################################
//
// FILE:	F2806x_MemCopy.c
//
// TITLE:	Memory Copy Utility
//
// ASSUMPTIONS:
//
//
//
// DESCRIPTION:
//
//          This function will copy the specified memory contents from
//          one location to another.
//
//          Uint16 *SourceAddr        Pointer to the first word to be moved
//                                    SourceAddr < SourceEndAddr
//          Uint16* SourceEndAddr     Pointer to the last word to be moved
//          Uint16* DestAddr          Pointer to the first destination word
//
//          No checks are made for invalid memory locations or that the
//          end address is > then the first start address.
//
//
//###########################################################################
// $TI Release: 2806x C/C++ Header Files and Peripheral Examples V1.00 $
// $Release Date: January 11, 2011 $
//###########################################################################

#include "F2806x_Device.h"

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}

//===========================================================================
// End of file.
//===========================================================================
