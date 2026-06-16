# BPA_State (0x602) 통신 규약 — SW 반영본

> 기준: F28069 PackBMS 펌웨어 (`SystemState_BIT`, `BATStatus_BIT`)
> 작성일: 2026-06-06 · 규약 R7 + SW 반영
> ECU: BRA · DLC: 8 byte · Cycle: 10 ms · Master R / SUB T · Byte Order: LSB(Intel)
> CAN ID: `0x602 | PackID`

전송 코드: [main.c](../SysSoure/main.c) 0x602 송신부
`CANATX(ID, 8, PackSateInfo, PackStatus.all, PackStateReg.Word.DataL, PackStateReg.Word.DataH)`

- Byte0~1 = `PackSateInfo` (Protect_Status<<8 | Device_Status)
- Byte2~3 = `PackStatus.all` (BATStatus_BIT, bit16~31)
- Byte4~7 = `PackStateReg.Word` (SystemState_BIT, bit32~63)

---

## Byte0~1 — 상태

| Signal | Len | Start | Type | Value Table | 코드 변수 |
|---|---|---|---|---|---|
| BRA_Divice_Status | 8 | 0 | Unsigned | 0:INIT,1:STANDBY,2:READY,3:RUNING,4:PROTECTER,5:DATALOG,6:ProtectHistory,7:MANUALMode | `PackStateReg.bit.SysSeqState` |
| BRA_Protect_Status | 8 | 8 | Unsigned | 0:None,1:Alarm,2:Fault,3:Protect | `CANARegs.PackProtetSate` |

## Byte2~3 — 릴레이/센서 상태 (BATStatus_BIT)

| Signal | Len | Start | Type | 코드 소스 |
|---|---|---|---|---|
| BRA_Balance | 1 | 16 | Unsigned | `PackStateReg.bit.SysBalanceEn` |
| BRA_Neg_Rly | 1 | 17 | Unsigned | `PrtectRelayRegs.State.bit.NRlyDI` |
| BRA_Pos_Rly | 1 | 18 | Unsigned | `PrtectRelayRegs.State.bit.PRlyDI` |
| BRA_PreChar_Rly | 1 | 19 | Unsigned | `PrtectRelayRegs.State.bit.ProRlyDI` |
| BRA_Neg_Rly_Aux | 1 | 20 | Unsigned | `PrtectRelayRegs.State.bit.NRlyDI` |
| BRA_Pos_Rly_Aux | 1 | 21 | Unsigned | `PrtectRelayRegs.State.bit.PRlyDI` |
| BRA_PreChar_AUX | 1 | 22 | Unsigned | `PrtectRelayRegs.State.bit.ProRlyDI` |
| BRA_MSD_AUX | 1 | 23 | Unsigned | `PackStateReg.bit.MSDERR` |
| BRA_Emg_STOP_SW | 1 | 24 | Unsigned | `DigitalInputReg.bit.EMGSWStauts` |
| BRA_Water_Leak | 1 | 25 | Unsigned | 모듈 `WaterleakFault` 종합 |
| BRA_OffGas | 1 | 26 | Unsigned | 0 (미구현) |
| (예비) | 5 | 27 | — | Stauts11~15 |

## Byte4~7 — 시스템 상태 (SystemState_BIT, bit32~63)

| Signal | Len | Start | Type | 코드 변수 (SystemState_BIT) |
|---|---|---|---|---|
| Sys_SysSeqState | 3 | 32 | Unsigned | SysSeqState |
| Sys_RlySeqState | 3 | 35 | Unsigned | RlySeqState |
| Sys_SocSeqState | 2 | 38 | Unsigned | SocSeqState |
| Sys_INITOK | 1 | 40 | Unsigned | INITOK |
| Sys_BalanceEn | 1 | 41 | Unsigned | SysBalanceEn |
| Sys_DisCharMode | 1 | 42 | Unsigned | SysDisCharMode |
| Sys_Alarm | 1 | 43 | Unsigned | SysAalarm |
| Sys_Fault | 1 | 44 | Unsigned | SysFault |
| Sys_Prtct | 1 | 45 | Unsigned | SysPrtct |
| Sys_CanEnable | 1 | 46 | Unsigned | CANCOMEnable |
| Sys_NRlyStatus | 1 | 47 | Unsigned | NRlyDOStatus |
| Sys_PRlyStatus | 1 | 48 | Unsigned | PRlyDOStatus |
| Sys_PreRlyStatus | 1 | 49 | Unsigned | PreRlyDOStatus |
| Sys_AdminMode | 1 | 50 | Unsigned | HMICOMMode |
| Sys_AdminBalaMode | 1 | 51 | Unsigned | HMIBalanceMode |
| Sys_MSDERR | 1 | 52 | Unsigned | MSDERR |
| Sys_RlyERR | 1 | 53 | Unsigned | RlyERR |
| Sys_PmsCANErr | 1 | 54 | Unsigned | SysPmsCANErr (신규, 미구현=0) |
| Sys_UnitComErr | 1 | 55 | Unsigned | SysUnitComErr (신규, 구 CANCOMERR 기능) |
| Sys_BATIC_Err | 1 | 56 | Unsigned | SysBATIC_Err (신규, 모듈 BATIC 종합) |
| Sys_VCURlyWakeUp | 1 | 57 | Unsigned | VCURlyWakeUp |
| Sys_ChargerWakeUp | 1 | 58 | Unsigned | ChargerWakeUp |
| Sys_SocMode | 1 | 59 | Unsigned | SysSocMode |
| Sys_BalaMode | 1 | 60 | Unsigned | SysBalaMode |
| Sys_PwrHoldRly | 1 | 61 | Unsigned | PwrHoldRlyDOStatus |
| Sys_CellVoltOk | 1 | 62 | Unsigned | CellVoltOk |
| Sys_CellTempsOk | 1 | 63 | Unsigned | CellTempsOk |

---

## R7 → SW 반영 변경 이력 (2026-06-06)

- **미적용**: Sys_UnitRead, Sys_PmsRead (규약 신규였으나 SW 미적용 → 후속 항목 2칸 당김)
- **삭제**: CANCOMERR(→ Sys_UnitComErr 기능 이전), ISOSPICOMERR, IMDRegErr
- **신규**: Sys_PmsCANErr(bit54), Sys_UnitComErr(bit55), Sys_BATIC_Err(bit56)
- **재활용**: HMICOMMode→Sys_AdminMode, HMIBalanceMode→Sys_AdminBalaMode
- **코드 전용 → 규약 반영**: VCURlyWakeUp, ChargerWakeUp, SocMode, BalaMode, PwrHoldRly, CellVoltOk, CellTempsOk (bit57~63)
