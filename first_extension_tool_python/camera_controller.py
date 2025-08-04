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
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux
from isaacsim.core.utils.stage import get_current_stage


class CameraController:
    """카메라 생성 및 관리 클래스"""
    
    def __init__(self):
        self.cameras = {}  # 생성된 카메라들을 저장
        self.stage = None
        self._update_stage()
    
    def _update_stage(self):
        """현재 스테이지 업데이트"""
        self.stage = get_current_stage()
    
    def create_basic_camera(self, camera_name: str, position: Gf.Vec3f = Gf.Vec3f(0, 0, 10), rotation: Gf.Rotation = None):
        """
        기본 카메라 생성
        
        Args:
            camera_name (str): 카메라 이름
            position (Gf.Vec3f): 카메라 위치 (x, y, z)
            rotation (Gf.Rotation): 카메라 회전 (기본값: None, 회전 없음)
        
        Returns:
            bool: 성공 여부
        """
        try:
            self._update_stage()
            
            # 카메라 경로 생성
            camera_path = f"/World/{camera_name}"
            
            # 카메라가 이미 존재하는지 확인
            if self.stage.GetPrimAtPath(camera_path).IsValid():
                print(f"카메라 '{camera_name}'이(가) 이미 존재합니다.")
                return False
            
            # 카메라 생성
            omni.kit.commands.execute(
                "CreatePrim",
                prim_type="Camera",
                prim_path=camera_path
            )
            
            # 카메라 속성 설정
            from pxr import UsdGeom
            camera_prim = self.stage.GetPrimAtPath(camera_path)
            camera = UsdGeom.Camera(camera_prim)
            
            # 위치 및 회전 설정 (기존 transform operation이 있는지 확인)
            xformable = UsdGeom.Xformable(camera_prim)
            translate_op = None
            rotate_op = None
            
            # 기존 transform operations 찾기
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeRotate:
                    rotate_op = op
            
            # translate operation이 없으면 새로 생성
            if translate_op is None:
                translate_op = xformable.AddTranslateOp()
            
            # rotate operation이 없으면 새로 생성
            if rotate_op is None and rotation is not None:
                rotate_op = xformable.AddRotateOp()
            
            # 위치 설정
            translate_op.Set(position)
            
            # 회전 설정 (있는 경우에만)
            if rotation is not None and rotate_op is not None:
                rotate_op.Set(rotation)
            
            # 카메라 속성 설정
            camera.CreateFocalLengthAttr().Set(24.0)
            camera.CreateHorizontalApertureAttr().Set(20.955)
            camera.CreateVerticalApertureAttr().Set(15.2908)
            
            # 카메라 정보 저장
            self.cameras[camera_name] = {
                "path": camera_path,
                "position": position,
                "rotation": rotation,
                "type": "basic"
            }
            
            if rotation is not None:
                print(f"카메라 '{camera_name}'이(가) 생성되었습니다. 위치: {position}, 회전: {rotation}")
            else:
                print(f"카메라 '{camera_name}'이(가) 생성되었습니다. 위치: {position}")
            return True
            
        except Exception as e:
            print(f"카메라 생성 중 오류 발생: {e}")
            return False
    
    def create_drone_camera(self, camera_name: str, position: Gf.Vec3f = Gf.Vec3f(0, 0, 10)):
        """
        드론 카메라 생성 (넓은 시야각)
        
        Args:
            camera_name (str): 카메라 이름
            position (Gf.Vec3f): 카메라 위치 (x, y, z)
        
        Returns:
            bool: 성공 여부
        """
        try:
            self._update_stage()
            
            # 카메라 경로 생성
            camera_path = f"/World/{camera_name}"
            
            # 카메라가 이미 존재하는지 확인
            if self.stage.GetPrimAtPath(camera_path).IsValid():
                print(f"카메라 '{camera_name}'이(가) 이미 존재합니다.")
                return False
            
            # 드론 카메라 생성 (넓은 시야각)
            omni.kit.commands.execute(
                "CreatePrim",
                prim_type="Camera",
                prim_path=camera_path
            )
            
            # 카메라 속성 설정
            from pxr import UsdGeom
            camera_prim = self.stage.GetPrimAtPath(camera_path)
            camera = UsdGeom.Camera(camera_prim)
            
            # 위치 설정 (기존 transform operation이 있는지 확인)
            xformable = UsdGeom.Xformable(camera_prim)
            translate_op = None
            
            # 기존 translate operation 찾기
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            
            # translate operation이 없으면 새로 생성
            if translate_op is None:
                translate_op = xformable.AddTranslateOp()
            
            # 위치 설정
            translate_op.Set(position)
            
            # 드론 카메라 속성 설정 (넓은 시야각)
            camera.CreateFocalLengthAttr().Set(16.0)  # 넓은 시야각을 위한 짧은 초점 거리
            camera.CreateHorizontalApertureAttr().Set(20.955)
            camera.CreateVerticalApertureAttr().Set(15.2908)
            
            # 카메라 정보 저장
            self.cameras[camera_name] = {
                "path": camera_path,
                "position": position,
                "type": "drone",
                "checkpoints": []  # 체크포인트 리스트
            }
            
            print(f"드론 카메라 '{camera_name}'이(가) 생성되었습니다. 위치: {position}")
            return True
            
        except Exception as e:
            print(f"드론 카메라 생성 중 오류 발생: {e}")
            return False
    
    def add_checkpoint(self, camera_name: str, position: Gf.Vec3f):
        """
        드론 카메라에 체크포인트 추가
        
        Args:
            camera_name (str): 카메라 이름
            position (Gf.Vec3f): 체크포인트 위치
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            self.cameras[camera_name]["checkpoints"].append(position)
            print(f"체크포인트가 추가되었습니다: {position}")
        else:
            print(f"드론 카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
    
    def clear_checkpoints(self, camera_name: str):
        """
        드론 카메라의 모든 체크포인트 제거
        
        Args:
            camera_name (str): 카메라 이름
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            self.cameras[camera_name]["checkpoints"].clear()
            print(f"드론 카메라 '{camera_name}'의 모든 체크포인트가 제거되었습니다.")
        else:
            print(f"드론 카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
    
    def get_checkpoints(self, camera_name: str):
        """
        드론 카메라의 체크포인트 목록 반환
        
        Args:
            camera_name (str): 카메라 이름
        
        Returns:
            list: 체크포인트 위치 리스트
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            return self.cameras[camera_name]["checkpoints"]
        else:
            print(f"드론 카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
            return []
    
    def remove_checkpoint(self, camera_name: str, index: int):
        """
        특정 인덱스의 체크포인트 제거
        
        Args:
            camera_name (str): 카메라 이름
            index (int): 제거할 체크포인트 인덱스
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            checkpoints = self.cameras[camera_name]["checkpoints"]
            if 0 <= index < len(checkpoints):
                removed_checkpoint = checkpoints.pop(index)
                print(f"체크포인트가 제거되었습니다: {removed_checkpoint}")
            else:
                print(f"인덱스 {index}가 유효하지 않습니다.")
        else:
            print(f"드론 카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
    
    def enable_checkpoint(self, camera_name: str, index: int, enabled: bool = True):
        """
        체크포인트 활성화/비활성화
        
        Args:
            camera_name (str): 카메라 이름
            index (int): 체크포인트 인덱스
            enabled (bool): 활성화 여부
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            if "checkpoint_states" not in self.cameras[camera_name]:
                self.cameras[camera_name]["checkpoint_states"] = {}
            
            checkpoints = self.cameras[camera_name]["checkpoints"]
            if 0 <= index < len(checkpoints):
                self.cameras[camera_name]["checkpoint_states"][index] = enabled
                status = "활성화" if enabled else "비활성화"
                print(f"체크포인트 {index}가 {status}되었습니다.")
            else:
                print(f"인덱스 {index}가 유효하지 않습니다.")
        else:
            print(f"드론 카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
    
    def is_checkpoint_enabled(self, camera_name: str, index: int) -> bool:
        """
        체크포인트 활성화 상태 확인
        
        Args:
            camera_name (str): 카메라 이름
            index (int): 체크포인트 인덱스
        
        Returns:
            bool: 활성화 여부 (기본값: True)
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            if "checkpoint_states" not in self.cameras[camera_name]:
                return True  # 기본값: 활성화
            
            return self.cameras[camera_name]["checkpoint_states"].get(index, True)
        else:
            return True  # 기본값: 활성화
    
    def get_enabled_checkpoints(self, camera_name: str):
        """
        활성화된 체크포인트만 반환
        
        Args:
            camera_name (str): 카메라 이름
        
        Returns:
            list: 활성화된 체크포인트 위치 리스트
        """
        if camera_name in self.cameras and self.cameras[camera_name]["type"] == "drone":
            checkpoints = self.cameras[camera_name]["checkpoints"]
            enabled_checkpoints = []
            
            for i, checkpoint in enumerate(checkpoints):
                if self.is_checkpoint_enabled(camera_name, i):
                    enabled_checkpoints.append(checkpoint)
            
            return enabled_checkpoints
        else:
            return []
    
    def remove_camera(self, camera_name: str):
        """
        카메라 제거
        
        Args:
            camera_name (str): 제거할 카메라 이름
        """
        try:
            if camera_name in self.cameras:
                camera_path = self.cameras[camera_name]["path"]
                omni.kit.commands.execute("DeletePrims", paths=[camera_path])
                del self.cameras[camera_name]
                print(f"카메라 '{camera_name}'이(가) 제거되었습니다.")
            else:
                print(f"카메라 '{camera_name}'을(를) 찾을 수 없습니다.")
        except Exception as e:
            print(f"카메라 제거 중 오류 발생: {e}")
    
    def get_camera_list(self):
        """
        생성된 카메라 목록 반환
        
        Returns:
            list: 카메라 이름 리스트
        """
        return list(self.cameras.keys())
    
    def get_camera_info(self, camera_name: str):
        """
        카메라 정보 반환
        
        Args:
            camera_name (str): 카메라 이름
        
        Returns:
            dict: 카메라 정보
        """
        return self.cameras.get(camera_name, None)
