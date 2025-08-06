# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import omni.kit.commands
import omni.timeline
from pxr import Gf, UsdGeom
from isaacsim.core.utils.stage import get_current_stage
from .bezier_path_generator import BezierPathGenerator
from .polynomial_path_generator import PolynomialPathGenerator


class DroneSimulator:
    """극값 기반 곡선 움직임을 구현하는 드론 시뮬레이션 클래스"""
    
    def __init__(self):
        self.stage = None
        self._update_stage()
        self.active_drones = {}  # 활성 드론들 관리
        self._timeline = omni.timeline.get_timeline_interface()
        
        # 극값 기반 곡선 경로 생성기 초기화
        self.path_generator = BezierPathGenerator(deviation_factor=0.05, speed_variation=0.05)
        # 다항식 곡선 경로 생성기 초기화 (새로운 알고리즘)
        self.polynomial_path_generator = PolynomialPathGenerator(points_per_segment=20)
    
    def _update_stage(self):
        """현재 스테이지 업데이트"""
        self.stage = get_current_stage()
    
    def start_drone_simulation(self, camera_name: str, checkpoints: list, speed: float = 2.0):
        """
        드론 시뮬레이션 시작 (극값 기반 곡선 방식)
        
        Args:
            camera_name (str): 드론 카메라 이름
            checkpoints (list): 체크포인트 위치 리스트 (극값 지점들)
            speed (float): 이동 속도 (m/s)
        """
        if not checkpoints:
            print("체크포인트가 없습니다.")
            return False
        
        try:
            self._update_stage()
            
            # 드론 카메라 경로
            camera_path = f"/World/{camera_name}"
            camera_prim = self.stage.GetPrimAtPath(camera_path)
            
            if not camera_prim.IsValid():
                print(f"드론 카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
                return False
            
            # 현재 드론 위치를 시작점으로 사용
            from pxr import Gf
            camera_xform = UsdGeom.Xformable(camera_prim)
            translate_op = None
            for op in camera_xform.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            
            if translate_op is None:
                print("드론 위치를 가져올 수 없습니다.")
                return False
            
            # Vec3d를 Vec3f로 변환
            start_position_vec3d = translate_op.Get()
            start_position = Gf.Vec3f(start_position_vec3d[0], start_position_vec3d[1], start_position_vec3d[2])
            
            # 다항식 곡선 경로 생성: 시작점(드론 생성 위치) + 체크포인트들
            path_points = [start_position] + checkpoints
            
            # 다항식 곡선 경로 생성 (새로운 알고리즘)
            extrema_path = self.polynomial_path_generator.generate_polynomial_path(path_points)
            speed_profile = self.polynomial_path_generator.get_speed_profile(speed)
            
            # 시뮬레이션 정보 저장
            self.active_drones[camera_name] = {
                "checkpoints": checkpoints,
                "extrema_path": extrema_path,
                "speed_profile": speed_profile,
                "current_path_index": 0,
                "current_position": extrema_path[0],
                "target_position": extrema_path[1] if len(extrema_path) > 1 else extrema_path[0],
                "is_moving": True,  # 시뮬레이션 시작 시 이동 상태로 설정
                "camera_path": camera_path,
                "checkpoint_reached": 0,  # 도달한 체크포인트 수
                "is_at_extrema": False,  # 극값(체크포인트)에 있는지 여부
                "extrema_pause_time": 0.0  # 극값에서 정지할 시간
            }
            
            # 경로 생성기 초기화
            self.path_generator.reset_path_index()
            
            # 첫 번째 경로 포인트로 이동
            self._move_drone_to_position(camera_name, extrema_path[0])
            
            print(f"극값 기반 곡선 경로 생성 완료: {len(extrema_path)} 개의 경로 포인트")
            print(f"드론 초기화: 시작 위치={start_position}, 첫 번째 체크포인트={checkpoints[0] if checkpoints else start_position}")
            print(f"드론 시뮬레이션이 시작되었습니다. 체크포인트 수: {len(checkpoints)}")
            return True
            
        except Exception as e:
            print(f"드론 시뮬레이션 시작 중 오류 발생: {e}")
            return False
    
    def stop_drone_simulation(self, camera_name: str):
        """
        드론 시뮬레이션 중지
        
        Args:
            camera_name (str): 드론 카메라 이름
        """
        if camera_name in self.active_drones:
            del self.active_drones[camera_name]
            print(f"드론 시뮬레이션이 중지되었습니다.")
    
    def update_drone_simulation(self, delta_time: float):
        """
        드론 시뮬레이션 업데이트 (physics step에서 호출)
        
        Args:
            delta_time (float): 시간 간격
        """
        if not self.active_drones:
            return
            
        for camera_name, drone_info in list(self.active_drones.items()):
            self._update_drone_movement(camera_name, drone_info, delta_time)
    
    def _update_drone_movement(self, camera_name: str, drone_info: dict, delta_time: float):
        """
        드론 이동 업데이트 (극값 기반 곡선 방식)
        
        Args:
            camera_name (str): 드론 카메라 이름
            drone_info (dict): 드론 정보
            delta_time (float): 시간 간격
        """
        extrema_path = drone_info["extrema_path"]
        speed_profile = drone_info["speed_profile"]
        current_path_index = drone_info["current_path_index"]
        
        # 현재 위치와 목표 위치
        current_pos = drone_info["current_position"]
        target_pos = drone_info["target_position"]
        

        
        # 현재 속도 (속도 프로파일에서 가져오기)
        if current_path_index < len(speed_profile):
            current_speed = speed_profile[current_path_index]
        else:
            current_speed = speed_profile[-1] if speed_profile else 2.0  # 기본 속도
        
        # 속도가 0이면 기본 속도 사용
        if current_speed <= 0:
            current_speed = 2.0
        
        # 목표 지점까지의 거리 계산
        distance_to_target = (Gf.Vec3f(target_pos) - Gf.Vec3f(current_pos)).GetLength()
        
        # 디버깅 정보 출력 (처음 몇 번만)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        
        if self._debug_counter < 10:
            print(f"Extrema Debug: Current={current_pos}, Target={target_pos}, Distance={distance_to_target:.3f}, Speed={current_speed:.2f}, PathIndex={current_path_index}")
            self._debug_counter += 1
        
        # 체크포인트 도달 확인 (정지하지 않고 계속 이동)
        checkpoint_reached = self._check_extrema_reached(drone_info, current_pos)
        if checkpoint_reached:
            # 이미 도달한 체크포인트인지 확인 (중복 방지)
            already_reached = drone_info.get("checkpoint_reached", 0)
            if checkpoint_reached > already_reached:
                drone_info["checkpoint_reached"] = checkpoint_reached
                print(f"체크포인트 {checkpoint_reached} 도달! 계속 이동합니다.")
        
        if distance_to_target < 0.15:  # 목표 지점에 도달 (더 정확하게)
            # 다음 경로 포인트로 이동
            next_path_index = current_path_index + 1
            
            if next_path_index < len(extrema_path):
                drone_info["current_path_index"] = next_path_index
                drone_info["target_position"] = extrema_path[next_path_index]
                
                print(f"경로 포인트 {current_path_index + 1} 도달. 다음 포인트 {next_path_index + 1}로 이동.")
            else:
                # 경로의 끝에 도달 - 드론 정지
                drone_info["is_moving"] = False
                print(f"경로 완료. 드론이 마지막 체크포인트에 도달하여 정지합니다.")
                return  # 더 이상 이동하지 않음
        
        # 드론이 정지 상태가 아닐 때만 이동
        if drone_info["is_moving"]:
            # 목표 지점으로 이동
            direction = (Gf.Vec3f(target_pos) - Gf.Vec3f(current_pos)).GetNormalized()
            movement = direction * current_speed * delta_time
            
            new_position = Gf.Vec3f(current_pos) + movement
            
            # 목표 지점을 넘어서지 않도록 조정
            if distance_to_target < current_speed * delta_time:
                # 목표 지점에 정확히 도달
                new_position = target_pos
            
            drone_info["current_position"] = new_position
            
            # 카메라 위치 업데이트
            self._move_drone_to_position(camera_name, new_position)
    
    def _check_extrema_reached(self, drone_info: dict, current_pos: Gf.Vec3f):
        """
        현재 위치가 극값(체크포인트)에 도달했는지 확인
        
        Args:
            drone_info (dict): 드론 정보
            current_pos (Gf.Vec3f): 현재 위치
        
        Returns:
            int: 도달한 체크포인트 번호 (0이면 도달하지 않음)
        """
        checkpoints = drone_info["checkpoints"]
        current_path_index = drone_info["current_path_index"]
        
        # 디버깅 정보 (처음 몇 번만)
        if not hasattr(self, '_checkpoint_debug_counter'):
            self._checkpoint_debug_counter = 0
        
        if self._checkpoint_debug_counter < 5:
            print(f"Checkpoint Debug: Current={current_pos}, PathIndex={current_path_index}")
            for i, cp in enumerate(checkpoints):
                distance = (Gf.Vec3f(cp) - Gf.Vec3f(current_pos)).GetLength()
                print(f"  CP {i+1}: {cp}, Distance={distance:.3f}")
            self._checkpoint_debug_counter += 1
        
        # 현재 경로 포인트가 체크포인트에 해당하는지 확인
        for i, checkpoint in enumerate(checkpoints):
            # 현재 위치가 체크포인트에 충분히 가까운지 확인
            distance_to_checkpoint = (Gf.Vec3f(checkpoint) - Gf.Vec3f(current_pos)).GetLength()
            if distance_to_checkpoint < 0.8:  # 체크포인트 도달 거리 (더 관대하게)
                # 이미 도달한 체크포인트인지 확인 (중복 방지)
                already_reached = drone_info.get("checkpoint_reached", 0)
                if i + 1 > already_reached:
                    return i + 1
        
        return 0
    
    def _move_drone_to_position(self, camera_name: str, position: Gf.Vec3f):
        """
        드론 카메라를 지정된 위치로 이동
        
        Args:
            camera_name (str): 드론 카메라 이름
            position (Gf.Vec3f): 이동할 위치
        """
        try:
            camera_path = f"/World/{camera_name}"
            camera_prim = self.stage.GetPrimAtPath(camera_path)
            
            if camera_prim.IsValid():
                camera = UsdGeom.Camera(camera_prim)
                xformable = UsdGeom.Xformable(camera_prim)
                
                # 기존 translate operation 찾기
                translate_op = None
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        translate_op = op
                        break
                
                # translate operation이 없으면 새로 생성
                if translate_op is None:
                    translate_op = xformable.AddTranslateOp()
                
                # 위치 설정
                translate_op.Set(position)
                
                # 디버깅 정보 (처음 몇 번만)
                if not hasattr(self, '_move_debug_counter'):
                    self._move_debug_counter = 0
                
                if self._move_debug_counter < 3:
                    print(f"Drone Move: {camera_name} -> {position}")
                    self._move_debug_counter += 1
                
            else:
                print(f"카메라 프림을 찾을 수 없습니다: {camera_path}")
                
        except Exception as e:
            print(f"드론 이동 중 오류 발생: {e}")
    
    def get_drone_status(self, camera_name: str):
        """
        드론 상태 정보 반환
        
        Args:
            camera_name (str): 드론 카메라 이름
        
        Returns:
            dict: 드론 상태 정보
        """
        if camera_name in self.active_drones:
            drone_info = self.active_drones[camera_name]
            current_path_index = drone_info["current_path_index"]
            extrema_path = drone_info["extrema_path"]
            checkpoint_reached = drone_info.get("checkpoint_reached", 0)
            is_at_extrema = drone_info.get("is_at_extrema", False)
            
            status_text = ""
            if drone_info.get("is_moving", True):
                if checkpoint_reached > 0:
                    status_text = f"Active - CP {checkpoint_reached}/{len(drone_info['checkpoints'])} 도달! (Path: {current_path_index + 1}/{len(extrema_path)})"
                else:
                    status_text = f"Active - CP {checkpoint_reached}/{len(drone_info['checkpoints'])} (Path: {current_path_index + 1}/{len(extrema_path)})"
            else:
                status_text = f"Stopped - CP {checkpoint_reached}/{len(drone_info['checkpoints'])} (마지막 체크포인트 도달)"
            
            return {
                "current_path_point": current_path_index + 1,
                "total_path_points": len(extrema_path),
                "current_checkpoint": checkpoint_reached,
                "total_checkpoints": len(drone_info["checkpoints"]),
                "current_position": drone_info["current_position"],
                "is_active": True,
                "is_moving": drone_info["is_moving"],
                "status_text": status_text
            }
        else:
            return {"is_active": False}
    
    def is_drone_active(self, camera_name: str):
        """
        드론이 활성 상태인지 확인
        
        Args:
            camera_name (str): 드론 카메라 이름
        
        Returns:
            bool: 활성 상태 여부
        """
        return camera_name in self.active_drones 