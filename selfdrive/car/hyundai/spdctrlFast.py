# 크루즈 버튼을 이용한 가감속 조건을 제 사용조건에 맞게 수정하였으나 잘될지 모르겠음. 머리 쥐나기 일보직전ㅠ. 개발하신 아톰님께 경의를 표합니다.

import math
import numpy as np
from common.numpy_fast import clip, interp

from selfdrive.car.hyundai.spdcontroller  import SpdController

import common.log as trace1


class SpdctrlFast(SpdController):
    def __init__(self, CP=None):
        super().__init__( CP )
        self.cv_Raio = 0.5
        self.cv_Dist = -5
        self.steer_mode = ""

    def update_lead(self, CS,  dRel, yRel, vRel):
        lead_set_speed = self.cruise_set_speed_kph
        lead_wait_cmd = 600

        #dRel, yRel, vRel = self.get_lead( sm, CS )
        if CS.lead_distance < 150:
            dRel = CS.lead_distance # 레이더 인식거리 학인되면 인식거리를 dRel(이온 차간간격)으로 치환
            vRel = CS.lead_objspd

        dst_lead_distance = (CS.clu_Vanz*self.cv_Raio)   # 유지 거리. 현재속도*0.5, 60 *0.5 = 30미터
        
        if dst_lead_distance > 100:
            dst_lead_distance = 100
        #elif dst_lead_distance < 15:
            #dst_lead_distance = 15

        if dRel < 150: #앞차와의 간격이 150미터 미만이면, 즉 앞차가 인식되면,
            self.time_no_lean = 0
            d_delta = dRel - dst_lead_distance  # d_delta = 앞차간격(이온값) - 앞차간격(레이더) 
            lead_objspd = vRel  # 선행차량 상대속도.
        else:
            d_delta = 0
            lead_objspd = 0
 
        # 가속을 위한 사용자 임의속도 설정 후의 상황
        if CS.driverAcc_time:
            lead_set_speed = CS.clu_Vanz
            self.seq_step_debug = "운전자가속"
            lead_wait_cmd = 15
        elif (CS.VSetDis >= 70 and lead_objspd < -30) or (lead_objspd < -40):
            self.seq_step_debug = "감속(-10)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 10, -10)  
        elif (CS.VSetDis >= 60 and lead_objspd < -30) or (lead_objspd < -35):
            self.seq_step_debug = "감속(-8)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 10, -8)  
        elif (CS.VSetDis >= 60 and lead_objspd < -20) or (lead_objspd < -25):
            self.seq_step_debug = "감속(-6)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 10, -6)  
        elif (CS.VSetDis >= 50 and lead_objspd < -15) or (lead_objspd < -20):
            self.seq_step_debug = "감속(-4)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 10, -4)

        # 거리 유지 조건
        elif d_delta < 0 and CS.clu_Vanz > 50: # 거리유지중 다른차가 갑자기 끼어든 상황, 차량레이더로 인식한 차간거리가 갱신전 이온 차간거리가 선 반영되어 d_delta값이 (-)가 된 상황임
            self.seq_step_debug = "앞차가까움"
            if lead_objspd >= 0:    # 끼어든 차와 속도가 같거나 끼어든 차가 가속하고 있는 상황
                if CS.VSetDis > (CS.clu_Vanz + 30):  #차량 크루즈 설정속도가 현재 차속+30 보다 큰 상황, 예를들면, 차량속도가 50인데, 크루즈 설정속도가 80보다 큰 상황
                    self.seq_step_debug = "거리유지중"
                    lead_wait_cmd = 15
                    lead_set_speed = CS.VSetDis - 1  # 크루즈 설정속도를 1씩 다운시킴
                    if lead_set_speed < 40:
                        lead_set_speed = 40
                else: #앞차 가속중인데, 크루즈속도가 이온설정속도보다 낮은경우 가속
                    self.seq_step_debug = "가속(+5)"
                    #lead_set_speed = int(CS.VSetDis)
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, 5)
            elif lead_objspd < -30 or (dRel < 60 and CS.clu_Vanz > 60 and lead_objspd < -5): # 끼어든 차가 급감속 하는 경우
                self.seq_step_debug = "감속(-5)"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 10, -5)
            elif lead_objspd < -20 or (dRel < 80 and CS.clu_Vanz > 80 and lead_objspd < -5):  # 끼어든 차가 급감속 하는 경우
                self.seq_step_debug = "감속(-4)"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 20, -4)
            elif lead_objspd < -10:
                self.seq_step_debug = "감속(-3)"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -3)
            elif lead_objspd < 0:
                self.seq_step_debug = "감속(-2)"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -2)
            else:
                self.seq_step_debug = "가속(+1)"
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 100, 1)

        # 선행차량이 멀리 있는 상태에서 감속 조건
        elif lead_objspd < -30 and dRel < 100:  #차간거리 100이하 상대속도 -30 미만
            self.seq_step_debug = "감속(-3)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 20, -3)
        elif lead_objspd < -20 and dRel < 50:  #거리 조건 추가
            self.seq_step_debug = "감속(-3)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 20, -3)
        elif lead_objspd < -10 and dRel < 30:  #거리 조건 추가:
            self.seq_step_debug = "감속(-2)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 30, -2)
        elif lead_objspd < -7:
            self.seq_step_debug = "감속(-2)"
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 30, -2)
        elif self.cruise_set_speed_kph > CS.clu_Vanz:  #이온설정속도가 차량설정속도보다 큰경우
            self.seq_step_debug = "가속"
            # 선행 차량이 가속하고 있으면.
            if dRel >= 150: # 감지범위 밖에 멀리 떨어져 있으면 천천히 가속
                if CS.clu_Vanz >= 60:
                   self.seq_step_debug = "가속(>60)"
                   lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, 1)
                else:
                   self.seq_step_debug = "가속(<60)"
                   lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, 2)
            elif lead_objspd > 2: # 처음출발시 선행차량 가속중일 때
                if CS.clu_Vanz >= 60 and d_delta > 0: # 차량속도60이상 기준거리이상 벌어졌을 때 조건
                    self.seq_step_debug = "가속(+2)"
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, 2)
                elif CS.clu_Vanz >= 20 and lead_set_speed <= 40: # 처음 출발시 빠른 가속을 위해 차량속도 20이상 넘을 시 40까지 설정속도 올린 후 대기(적용시간을 벌기 위함)
                    self.seq_step_debug = "초반가속"
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 5, 10)
                elif CS.clu_Vanz >= 35 and lead_set_speed <= 50: # 차량속도 35넘을 때 50까지 설정속도 빠르게 올린 후 대기
                    self.seq_step_debug = "중반가속"
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 5, 10)
                elif CS.clu_Vanz >= 45 and lead_set_speed <= 60: # 차량속도 45넘을 때 60까지 설정속도 빠르게 올린 후 대기
                    self.seq_step_debug = "종반가속"
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 5, 10)
            else:
                if d_delta > 0: # 앞차가 기준거리(dst_lead_distance)보다 앞에 있을 때 일반 가속, 기준거리 안에 있을 때 가속하지 않음
                    self.seq_step_debug = "가속(+1)"
                    lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 50, 1)

        return lead_wait_cmd, lead_set_speed

    def update_curv(self, CS, sm, model_speed):
        wait_time_cmd = 0
        set_speed = self.cruise_set_speed_kph

        # 2. 커브 감속.
        #if self.cruise_set_speed_kph >= 100:
        if CS.clu_Vanz >= 60 and CS.out.cruiseState.modeSel == 1:
            if model_speed < 60:
                set_speed = self.cruise_set_speed_kph - 20
                self.seq_step_debug = "커브감속(-20)"
                wait_time_cmd = 300
            elif model_speed < 70:
                set_speed = self.cruise_set_speed_kph - 15
                self.seq_step_debug = "커브감속(-15)"
                wait_time_cmd = 250
            elif model_speed < 80:
                set_speed = self.cruise_set_speed_kph - 10
                self.seq_step_debug = "커브감속(-10)"
                wait_time_cmd = 200
            elif model_speed < 90:
                set_speed = self.cruise_set_speed_kph - 5
                self.seq_step_debug = "커브감속(-5)"
                wait_time_cmd = 150

        return wait_time_cmd, set_speed


    def update_log(self, CS, set_speed, target_set_speed, long_wait_cmd ):
        if CS.out.cruiseState.modeSel == 0:
            self.steer_mode = "오파모드"
        elif CS.out.cruiseState.modeSel == 1:
            self.steer_mode = "차간+커브"
        elif CS.out.cruiseState.modeSel == 2:
            self.steer_mode = "차간ONLY"
        elif CS.out.cruiseState.modeSel == 3:
            self.steer_mode = "자동RES"
        elif CS.out.cruiseState.modeSel == 4:
            self.steer_mode = "순정모드"
        str3 = '주행모드={:s}  설정속도={:03.0f}/{:03.0f}  타이머={:03.0f}/{:03.0f}/{:03.0f}'.format( self.steer_mode,
            set_speed,  CS.VSetDis, CS.driverAcc_time, long_wait_cmd, self.long_curv_timer )
        str4 = '  거리차/속도차={:03.0f}/{:03.0f}  구분={:s}'.format(  CS.lead_distance, CS.lead_objspd, self.seq_step_debug )

        str5 = str3 +  str4
        trace1.printf2( str5 )