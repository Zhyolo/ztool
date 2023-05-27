#ifndef __AXP173_REG_H__
#define __AXP173_REG_H__


// 电源控制类寄存器
#define AXP173_POWER_STATUS_REG                     0x00    // 电源状态寄存器
#define AXP173_POWER_MODE_CHGSTATUS_REG             0x01    // 电源模式/充电状态寄存器
#define AXP173_OTG_VBUS_STATUS_REG                  0x04    // OTG VBUS状态寄存器
#define AXP173_DATA_BUFFER0_REG                     0x06    // 数据缓冲区0
#define AXP173_DATA_BUFFER1_REG                     0x07    // 数据缓冲区1
#define AXP173_DATA_BUFFER2_REG                     0x08    // 数据缓冲区2
#define AXP173_DATA_BUFFER3_REG                     0x09    // 数据缓冲区3
#define AXP173_DATA_BUFFER4_REG                     0x0A    // 数据缓冲区4
#define AXP173_DATA_BUFFER5_REG                     0x0B    // 数据缓冲区5
#define AXP173_EXTEN_DC2_CTL_REG                    0x10    // EXTEN & DC-DC2 开关控制寄存器
#define AXP173_DC1_LDO234_CTL_REG                   0x12    // DC-DC1/LDO4 & LDO2/3 开关控制寄存器
#define AXP173_DC2_VOLTAGE_SET_REG                  0x23    // DC-DC2 输出电压设置寄存器
#define AXP173_DC2_SLEW_RATE_SET_REG                0x25    // DC-DC2 输出电压斜率设置寄存器
#define AXP173_DC1_VOLTAGE_SET_REG                  0x26    // DC-DC1 输出电压设置寄存器
#define AXP173_LDO4_VOLTAGE_SET_REG                 0x27    // LDO4 输出电压设置寄存器
#define AXP173_LDO23_VOLTAGE_SET_REG                0x28    // LDO2/3 输出电压设置寄存器
#define AXP173_VBUS_IPSOUT_CHANNEL_REG              0x30    // VBUS/IPsout 通路设置寄存器
#define AXP173_SHUTDOWN_VOLTAGE_SET_REG             0x31    // 关机电压设置寄存器
#define AXP173_SHUTDOWN_BAT_CHARGELED_CTL_REG       0x32    // 关机、电池检测、CHGLED 控制寄存器
#define AXP173_CHARGE1_CTL_REG                      0x33    // 充电控制寄存器1
#define AXP173_CHARGE2_CTL_REG                      0x34    // 充电控制寄存器2
#define AXP173_PEK_SET_REG                          0x36    // PEK 设置寄存器
#define AXP173_DCDC_FREQSET_REG                     0x37    // DC-DC 开关频率设置寄存器
#define AXP173_CHARGE_LOW_TEMP_ALARM_SET_REG        0x38    // 充电低温报警设置寄存器
#define AXP173_CHARGE_HIGH_TEMP_ALARM_SET_REG       0x39    // 充电高温报警设置寄存器
#define AXP173_APS_LOW_POWER_L1_SET_REG             0x3A    // APS 低电 Level1 设置寄存器
#define AXP173_APS_LOW_POWER_L2_SET_REG             0x3B    // APS 低电 Level2 设置寄存器
#define AXP173_DISCHARGE_LOW_TEMP_ALARM_SET_REG     0x3C    // 放电低温报警设置寄存器
#define AXP173_DISCHARGE_HIGH_TEMP_ALARM_SET_REG    0x3D    // 放电高温报警设置寄存器
#define AXP173_DC_MODE_SET_REG                      0x80    // DCDC 工作模式设置寄存器
#define AXP173_ADC_EN_CTL1_REG                      0x82    // ADC 使能控制寄存器1
#define AXP173_ADC_EN_CTL2_REG                      0x83    // ADC 使能控制寄存器2
#define AXP173_ADC_RATE_TS_PIN_CTL_REG              0x84    // ADC 速率、TS Pin 设置寄存器
#define AXP173_TIMER_CTL_REG                        0x8A    // 定时器控制寄存器
#define AXP173_VBUS_MONITOR_CTL_REG                 0x8B    // VBUS 监测控制寄存器
#define AXP173_OVER_TEMP_SHUTDOWN_CTL_REG           0x8F    // 过温关机控制寄存器

// 中断控制类寄存器
#define AXP173_INT_EN1_REG                          0x40    // 中断使能控制寄存器1
#define AXP173_INT_EN2_REG                          0x41    // 中断使能控制寄存器2
#define AXP173_INT_EN3_REG                          0x42    // 中断使能控制寄存器3
#define AXP173_INT_EN4_REG                          0x43    // 中断使能控制寄存器4
#define AXP173_STATUS_1_REG                         0x44    // 中断状态寄存器1
#define AXP173_STATUS_2_REG                         0x45    // 中断状态寄存器2
#define AXP173_STATUS_3_REG                         0x46    // 中断状态寄存器3
#define AXP173_STATUS_4_REG                         0x47    // 中断状态寄存器4

// ADC 数据寄存器
#define AXP173_ACIN_VOL_H8_REG                      0x56    // ACIN 电压高8位
#define AXP173_ACIN_VOL_L4_REG                      0x57    // ACIN 电压低4位
#define AXP173_ACIN_CUR_H8_REG                      0x58    // ACIN 电流高8位
#define AXP173_ACIN_CUR_L4_REG                      0x59    // ACIN 电流低4位
#define AXP173_VBUS_VOL_H8_REG                      0x5A    // VBUS 电压高8位
#define AXP173_VBUS_VOL_L4_REG                      0x5B    // VBUS 电压低4位
#define AXP173_VBUS_CUR_H8_REG                      0x5C    // VBUS 电流高8位
#define AXP173_VBUS_CUR_L4_REG                      0x5D    // VBUS 电流低4位
#define AXP173_TEMP_H8_REG                          0x5E    // 芯片温度高8位
#define AXP173_TEMP_L4_REG                          0x5F    // 芯片温度低4位
#define AXP173_TS_IN_H8_REG                         0x62    // TS 输入电压高8位
#define AXP173_TS_IN_L4_REG                         0x63    // TS 输入电压低4位
#define AXP173_BAT_POWERH8_REG                      0x70    // 电池瞬时功率高 8 位
#define AXP173_BAT_POWERM8_REG                      0x71    // 电池瞬时功率中 8 位
#define AXP173_BAT_POWERL8_REG                      0x72    // 电池瞬时功率低 8 位
#define AXP173_BAT_VOL_H8_REG                       0x78    // 电池电压高 8 位
#define AXP173_BAT_VOL_L4_REG                       0x79    // 电池电压低 4 位
#define AXP173_CHARGE_CUR_H8_REG                    0x7A    // 充电电流高 8 位
#define AXP173_CHARGE_CUR_L5_REG                    0x7B    // 充电电流低 5 位
#define AXP173_DISCHARGE_CUR_H8_REG                 0x7C    // 放电电流高 8 位
#define AXP173_DISCHARGE_CUR_L5_REG                 0x7D    // 放电电流低 5 位
#define AXP173_APS_VOL_H8_REG                       0x7E    // APS 电压高 8 位
#define AXP173_APS_VOL_L4_REG                       0x7F    // APS 电压低 4 位

// 电池电量寄存器
#define AXP173_CHARGE_COULOMB3_REG                  0xB0    // 电池充电库仑计数据寄存器 3
#define AXP173_CHARGE_COULOMB2_REG                  0xB1    // 电池充电库仑计数据寄存器 2
#define AXP173_CHARGE_COULOMB1_REG                  0xB2    // 电池充电库仑计数据寄存器 1
#define AXP173_CHARGE_COULOMB0_REG                  0xB3    // 电池充电库仑计数据寄存器 0
#define AXP173_DISCHARGE_COULOMB3_REG               0xB4    // 电池放电库仑计数据寄存器 3
#define AXP173_DISCHARGE_COULOMB2_REG               0xB5    // 电池放电库仑计数据寄存器 2
#define AXP173_DISCHARGE_COULOMB1_REG               0xB6    // 电池放电库仑计数据寄存器 1
#define AXP173_DISCHARGE_COULOMB0_REG               0xB7    // 电池放电库仑计数据寄存器 0
#define AXP173_COULOMB_CTL_REG                      0xB8    // 库仑计控制寄存器

#endif /* __AXP173_REG_H__ */