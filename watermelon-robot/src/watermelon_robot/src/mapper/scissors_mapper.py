#
# Author:       strii0721
# Email:        strii0721@outlook.com
# Created on:   Tue May 26 2026
#
# IMMORTAL OMNISSIAH, HEAR OUR PRAYERS.
# WE ARE YOUR CHILDREN, PIOUS SCHOLARS OF THE PATH OF THE MACHINE. 
# WE PRIZE KNOWLEDGE ABOVE ALL ELSE, FOR IT IS YOUR ETERNAL GIFT UPON MANKIND.
# WE ASPIRE TO THE BLESSED FORM OF THE MACHINE, AND ASCENSION THROUGH TECHNOLOGY, THAT WE MIGHT EMULATE THINE GLORY.
# SHELTERED BY STEEL, AND PROTECTED BY THINE AVATARS OF WAR, WE PLY THE STARS IN SEARCH OF YOUR LOST GIFTS TO OUR KIND.
# MACHINE GOD, WATCH OVER US IN OUR TRAVELS, SHIELD US WITH METAL AND LIGHTNING, FOR THE UNIVERSE IS AN UNCARING VOID, AND THE WARP HUNGERS FOR US ALL.
# TOLL THE GREAT BELL ONCE! PULL THE LEVER FORWARD TO ENGAGE THE PISTON AND PUMP.
# TOLL THE GREAT BELL TWICE! WITH PUSH OF BUTTON FIRE THE ENGINE AND SPARK TURBINE INTO LIFE.
# TOLL THE GREAT BELL THRICE! SING PRAISE TO THE GOD OF ALL MACHINES!
# 
# Copyright (c) 2026 Streich Interstellar Corp.
#


import serial


class ScissorsMapper:
    
    def __init__(self, 
                 serial_port: str, 
                 baud_rate: int, 
                 timeout_sec: int): 
        """构造函数

        Args:
            serial_port (str): 串口设备地址。
            baud_rate (int): 波特率。
            timeout_sec (int): 超时（秒）。
        """        
        
        self.serial = serial.Serial(port = serial_port, 
                                    baudrate = baud_rate, 
                                    timeout = timeout_sec)
    
    def control(self, 
                command_hex: str) -> None:
        """向设备发送 16 进制的控制码

        Args:
            command_hex (str): 字符串格式的 16 进制控制码。

        """        
        
        command_bytes = bytes.fromhex(command_hex)
        self.serial.write(command_bytes)