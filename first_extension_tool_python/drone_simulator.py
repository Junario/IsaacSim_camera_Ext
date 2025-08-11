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
        드론 이동 업데이트 (위치 + 방향)
        
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
            
            # 1. 카메라 위치 업데이트 (translate)
            self._move_drone_to_position(camera_name, new_position)
            
            # 2. 카메라 방향 업데이트 (orient) - 새로 추가
            self._update_camera_orientation_based_on_tangent(
                camera_name, new_position, extrema_path
            )
            
            drone_info["current_position"] = new_position
    
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
    
    # 접선 기반 카메라 회전 함수들 추가
    def _calculate_tangent_at_position(self, current_pos: Gf.Vec3f, path_points: list, tangent_window: float = 2.0):
        """
        현재 위치에서 경로의 접선 벡터를 계산 (개선된 버전)
        
        Args:
            current_pos: 현재 카메라 위치
            path_points: 전체 경로 포인트들
            tangent_window: 접선 계산을 위한 윈도우 크기 (미터)
        
        Returns:
            Gf.Vec3f: 정규화된 접선 벡터
        """
        # 현재 위치에서 가장 가까운 경로 인덱스 찾기
        nearest_idx = self._find_nearest_path_index(current_pos, path_points)
        
        print(f"현재 위치: {current_pos}, 가장 가까운 인덱스: {nearest_idx}/{len(path_points)}")
        
        # 더 정확한 접선 계산: 현재 위치에서 미래 지점으로의 방향
        if nearest_idx < len(path_points) - 1:
            # 현재 위치에서 다음 지점까지의 방향
            next_point = path_points[nearest_idx + 1]
            direction = next_point - current_pos
            
            # 만약 현재 위치가 다음 지점에 너무 가까우면 더 먼 지점 사용
            if direction.GetLength() < 0.1:
                # 더 먼 지점 찾기
                for i in range(nearest_idx + 2, min(len(path_points), nearest_idx + 5)):
                    if i < len(path_points):
                        far_point = path_points[i]
                        direction = far_point - current_pos
                        if direction.GetLength() > 0.5:  # 충분한 거리가 있는 지점
                            break
            
            tangent_vector = direction.GetNormalized()
            print(f"접선 벡터 (다음 지점 기준): {direction} -> {tangent_vector}")
            
        else:
            # 마지막 지점인 경우 이전 지점으로부터의 방향 사용
            if nearest_idx > 0:
                prev_point = path_points[nearest_idx - 1]
                direction = current_pos - prev_point
                tangent_vector = direction.GetNormalized()
                print(f"마지막 지점 접선 벡터: {direction} -> {tangent_vector}")
            else:
                # 첫 번째 지점인 경우 기본 방향 사용
                tangent_vector = Gf.Vec3f(0, 0, 1)
                print(f"첫 번째 지점 기본 방향: {tangent_vector}")
        
        return tangent_vector
    
    def _find_nearest_path_index(self, current_pos: Gf.Vec3f, path_points: list):
        """
        현재 위치에서 가장 가까운 경로 인덱스 찾기
        """
        min_distance = float('inf')
        nearest_idx = 0
        
        for i, point in enumerate(path_points):
            distance = (point - current_pos).GetLength()
            if distance < min_distance:
                min_distance = distance
                nearest_idx = i
        
        return nearest_idx
    
    def _tangent_to_quaternion(self, tangent_vector: Gf.Vec3f, up_vector: Gf.Vec3f = Gf.Vec3f(0, 0, 1)):
        """
        접선 벡터를 기반으로 카메라 쿼터니언 생성 (카메라가 진행 방향을 바라보도록)
        
        Args:
            tangent_vector: 정규화된 접선 벡터 (진행 방향)
            up_vector: 카메라의 상향 벡터 (기본값: Z축)
        
        Returns:
            Gf.Quatd: 카메라 회전 쿼터니언
        """
        import math
        
        print(f"접선 벡터를 쿼터니언으로 변환: {tangent_vector}")
        
        # 카메라가 진행 방향을 바라보도록 하려면:
        # 1. 카메라의 전방 벡터가 접선 벡터와 일치하도록 회전
        # 2. 카메라의 상향 벡터는 up_vector와 일치하도록 유지
        
        # 기본 카메라 방향 (Isaac Sim에서는 -Z축이 전방)
        camera_forward = Gf.Vec3f(0, 0, -1)
        camera_up = Gf.Vec3f(0, 1, 0)
        
        # 접선 벡터가 Z축과 평행한 경우 특별 처리
        if abs(tangent_vector[2]) > 0.99:
            if tangent_vector[2] > 0:
                # Z축 정방향 (Isaac Sim에서는 -Z가 전방이므로 180도 회전)
                quat = Gf.Quatd(0, 1, 0, 0)
            else:
                # Z축 역방향 (Isaac Sim에서는 -Z가 전방이므로 기본 방향)
                quat = Gf.Quatd(1, 0, 0, 0)
            print(f"수직 이동 쿼터니언: {quat}")
            return quat
        
        # 접선 벡터를 카메라 전방으로 회전시키는 쿼터니언 계산
        # 1. 회전축 계산 (카메라 전방과 접선 벡터의 외적)
        rotation_axis = Gf.Cross(camera_forward, tangent_vector)
        
        if rotation_axis.GetLength() < 1e-6:
            # 이미 같은 방향인 경우
            quat = Gf.Quatd(1, 0, 0, 0)
            print(f"이미 같은 방향 쿼터니언: {quat}")
            return quat
        
        rotation_axis = rotation_axis.GetNormalized()
        
        # 2. 회전각 계산
        cos_angle = Gf.Dot(camera_forward, tangent_vector)
        angle = math.acos(max(-1, min(1, cos_angle)))
        
        # 3. 쿼터니언 생성
        quat = Gf.Quatd(math.sin(angle/2) * rotation_axis[0],
                         math.sin(angle/2) * rotation_axis[1],
                         math.sin(angle/2) * rotation_axis[2],
                         math.cos(angle/2))
        
        print(f"계산된 쿼터니언 (진행 방향): {quat}")
        print(f"회전축: {rotation_axis}, 회전각: {math.degrees(angle):.2f}도")
        
        return quat
    
    def _update_camera_rotation(self, camera_name: str, rotation: Gf.Quatd):
        """
        카메라 회전 업데이트
        
        Args:
            camera_name: 카메라 이름
            rotation: 새로운 회전 쿼터니언 (Gf.Quatd)
        """
        try:
            camera_path = f"/World/{camera_name}"
            camera_prim = self.stage.GetPrimAtPath(camera_path)
            
            if camera_prim.IsValid():
                xformable = UsdGeom.Xformable(camera_prim)
                
                # 기존 rotate operation 찾기
                rotate_op = None
                for op in xformable.GetOrderedXformOps():
                    if op.GetOpType() in [UsdGeom.XformOp.TypeRotateX, UsdGeom.XformOp.TypeRotateY, 
                                         UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeOrient]:
                        rotate_op = op
                        break
                
                # rotate operation이 없으면 새로 생성 (Orient 타입 사용)
                if rotate_op is None:
                    rotate_op = xformable.AddOrientOp()
                
                # 회전 값 설정 (Gf.Quatd 타입으로)
                rotate_op.Set(rotation)
                
                # 디버깅 정보 (처음 몇 번만)
                if not hasattr(self, '_rotation_debug_counter'):
                    self._rotation_debug_counter = 0
                
                if self._rotation_debug_counter < 5:
                    print(f"Camera Rotation Update: {camera_name} -> {rotation}")
                    self._rotation_debug_counter += 1
                
            else:
                print(f"카메라 프림을 찾을 수 없습니다: {camera_path}")
                
        except Exception as e:
            print(f"카메라 회전 업데이트 중 오류 발생: {e}")
    
    def _calculate_dynamic_smoothing_factor(self, speed: float, base_factor: float = 0.3) -> float:
        """
        속도에 따른 동적 보간 계수 계산
        
        Args:
            speed: 현재 드론 속도 (m/s)
            base_factor: 기본 보간 계수 (0.3)
        
        Returns:
            float: 속도에 맞게 조정된 보간 계수
        """
        # 속도가 빠를수록 더 큰 보간 계수 사용 (부드러운 회전)
        if speed <= 1.0:
            # 느린 속도: 기본값 사용
            return base_factor
        elif speed <= 3.0:
            # 중간 속도: 선형 증가
            speed_ratio = (speed - 1.0) / 2.0
            return base_factor + (0.6 - base_factor) * speed_ratio
        else:
            # 빠른 속도: 최대값 사용
            return 0.6
    
    def _update_camera_orientation_based_on_tangent(self, camera_name: str, current_pos: Gf.Vec3f, 
                                                   path_points: list, smoothing_factor: float = None):
        """
        접선 기반으로 카메라 방향 업데이트
        
        Args:
            camera_name: 카메라 이름
            current_pos: 현재 카메라 위치
            path_points: 경로 포인트들
            smoothing_factor: 부드러운 보간 계수 (None이면 자동 계산)
        """
        try:
            print(f"=== 카메라 방향 업데이트 시작 ===")
            print(f"카메라: {camera_name}")
            print(f"현재 위치: {current_pos}")
            print(f"경로 포인트 수: {len(path_points)}")
            
            # 1. 현재 위치에서 접선 벡터 계산
            tangent_vector = self._calculate_tangent_at_position(current_pos, path_points)
            print(f"계산된 접선 벡터: {tangent_vector}")
            
            # 2. 접선 벡터를 쿼터니언으로 변환
            target_quat = self._tangent_to_quaternion(tangent_vector)
            print(f"목표 쿼터니언: {target_quat}")
            
            # 3. 현재 카메라 회전 가져오기
            current_quat = self._get_current_camera_rotation(camera_name)
            print(f"현재 쿼터니언: {current_quat}")
            
            # 4. 동적 보간 계수 계산 (속도에 따라 자동 조정)
            if smoothing_factor is None:
                # 현재 드론의 속도 정보 가져오기
                current_speed = 2.0  # 기본값 (실제로는 drone_info에서 가져와야 함)
                # 드론 정보에서 현재 속도 찾기
                for drone_name, drone_info in self.active_drones.items():
                    if drone_name == camera_name:
                        current_speed = drone_info.get("speed", 2.0)
                        break
                
                dynamic_smoothing_factor = self._calculate_dynamic_smoothing_factor(current_speed)
                print(f"현재 속도: {current_speed}m/s, 동적 보간 계수: {dynamic_smoothing_factor:.3f}")
            else:
                dynamic_smoothing_factor = smoothing_factor
                print(f"사용자 지정 보간 계수: {dynamic_smoothing_factor:.3f}")
            
            # 5. 부드러운 보간 적용 (동적 계수 사용)
            smooth_quat = self._smooth_orientation_interpolation(current_quat, target_quat, dynamic_smoothing_factor)
            print(f"보간된 쿼터니언: {smooth_quat}")
            
            # 6. 카메라 회전 업데이트
            self._update_camera_rotation(camera_name, smooth_quat)
            
            print(f"=== 카메라 방향 업데이트 완료 ===")
            return True
            
        except Exception as e:
            print(f"카메라 방향 업데이트 실패: {e}")
            return False
    
    def _get_current_camera_rotation(self, camera_name: str):
        """
        현재 카메라 회전 값 가져오기
        
        Args:
            camera_name (str): 카메라 이름
        
        Returns:
            Gf.Quatd: 현재 회전 값
        """
        try:
            camera_path = f"/World/{camera_name}"
            camera_prim = self.stage.GetPrimAtPath(camera_path)
            
            if not camera_prim.IsValid():
                return Gf.Quatd(1, 0, 0, 0)  # 기본값을 Quatd로 변경
            
            # Xformable 가져오기
            xformable = UsdGeom.Xformable(camera_prim)
            
            # 회전 operation 찾기
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() in [UsdGeom.XformOp.TypeRotateX, UsdGeom.XformOp.TypeRotateY, 
                                     UsdGeom.XformOp.TypeRotateZ, UsdGeom.XformOp.TypeOrient]:
                    current_quat = op.Get()
                    # 모든 쿼터니언을 Gf.Quatd로 변환
                    if hasattr(current_quat, 'GetImaginary') and hasattr(current_quat, 'GetReal'):
                        # 이미 Gf.Quatd인 경우 그대로 반환
                        return current_quat
                    else:
                        # Gf.Quatf인 경우 Gf.Quatd로 변환
                        return Gf.Quatd(current_quat.GetImaginary()[0], 
                                       current_quat.GetImaginary()[1], 
                                       current_quat.GetImaginary()[2], 
                                       current_quat.GetReal())
            
            # 회전 operation이 없으면 기본값 반환
            return Gf.Quatd(1, 0, 0, 0)  # 단위 쿼터니언 (Quatd)
            
        except Exception as e:
            print(f"카메라 회전 값 가져오기 실패: {e}")
            return Gf.Quatd(1, 0, 0, 0)
    
    def _smooth_orientation_interpolation(self, current_quat: Gf.Quatd, target_quat: Gf.Quatd, 
                                        smoothing_factor: float = 0.4):
        """
        현재 회전과 목표 회전 사이를 부드럽게 보간
        
        Args:
            current_quat: 현재 카메라 쿼터니언 (Gf.Quatd)
            target_quat: 목표 쿼터니언 (Gf.Quatd)
            smoothing_factor: 보간 계수 (0~1, 작을수록 부드러움, 기본값: 0.4)
        
        Returns:
            Gf.Quatd: 보간된 쿼터니언
        """
        try:
            # 타입 확인 및 변환 - 모든 쿼터니언을 Gf.Quatd로 통일
            if not isinstance(current_quat, Gf.Quatd):
                if hasattr(current_quat, 'GetImaginary') and hasattr(current_quat, 'GetReal'):
                    # Gf.Quatf를 Gf.Quatd로 변환
                    current_quat = Gf.Quatd(current_quat.GetImaginary()[0], 
                                           current_quat.GetImaginary()[1], 
                                           current_quat.GetImaginary()[2], 
                                           current_quat.GetReal())
            
            if not isinstance(target_quat, Gf.Quatd):
                if hasattr(target_quat, 'GetImaginary') and hasattr(target_quat, 'GetReal'):
                    # Gf.Quatf를 Gf.Quatd로 변환
                    target_quat = Gf.Quatd(target_quat.GetImaginary()[0], 
                                          target_quat.GetImaginary()[1], 
                                          target_quat.GetImaginary()[2], 
                                          target_quat.GetReal())
            
            # Slerp 함수 호출 수정 - 올바른 시그니처 사용
            interpolated_quat = Gf.Slerp(smoothing_factor, current_quat, target_quat)
            return interpolated_quat
            
        except Exception as e:
            print(f"쿼터니언 보간 실패: {e}")
            # 오류 발생 시 목표 쿼터니언 반환
            return target_quat 