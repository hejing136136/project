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


from mapper import ScissorsMapper
import time
from types import SimpleNamespace


class ScissorsService:
    
    def __init__(self, 
                 serial_port: str, 
                 baud_rate: int, 
                 timeout_sec: int, 
                 command_code_set: SimpleNamespace):
        """向上提供剪刀相关业务的类。

        Args:
            serial_port (str): 串口设备地址。
            baud_rate (int): 波特率。
            timeout_sec (int): 超时时长（秒）。
            command_code_set (SimpleNamespace): 控制码集。
        """        
        
        self.scissors_mapper = ScissorsMapper(serial_port = serial_port, 
                                              baud_rate = baud_rate, 
                                              timeout_sec = timeout_sec)
        
        self.command_code_set = command_code_set
        
    def cut_it(self, 
               operation_delay_sec: int) -> tuple:
        
        is_success = 0
        message = ""
        try:
            self.scissors_mapper.control(command_hex = self.command_code_set.close)
            time.sleep(operation_delay_sec)
            self.scissors_mapper.control(command_hex = self.command_code_set.open)
        except Exception as exception:
            is_success = -1
            message = f"{exception}"
        finally:
            return (is_success, message)