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

import omni.timeline
import omni.ui as ui
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux

from .scenario import FrankaRmpFlowExampleScript
from .camera_controller import CameraController
from .drone_simulator import DroneSimulator


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # 카메라 컨트롤러 초기화
        self.camera_controller = CameraController()
        
        # 드론 시뮬레이터 초기화
        self.drone_simulator = DroneSimulator()

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
            # For complete robustness, the user should resolve those edge cases here
            # In general, for extensions based off this template, there is no value to having the user click the play/stop
            # button instead of using the Load/Reset/Run buttons provided.
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        # 디버깅 정보 (처음 몇 번만)
        if not hasattr(self, '_physics_debug_counter'):
            self._physics_debug_counter = 0
        
        if self._physics_debug_counter < 10:
            print(f"Physics Step: {step}, Active Drones: {len(self.drone_simulator.active_drones)}")
            self._physics_debug_counter += 1
        
        # 드론 시뮬레이션 업데이트
        self.drone_simulator.update_drone_simulation(step)
        
        # 드론 상태 업데이트 (UI에서 확인 가능)
        if hasattr(self, '_drone_status_label'):
            self._update_drone_status()

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = LoadButton(
                    "Load Button", "LOAD", setup_scene_fn=self._setup_scene, setup_post_load_fn=self._setup_scenario
                )
                self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        run_scenario_frame = CollapsableFrame("Run Scenario")

        with run_scenario_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._on_run_scenario_a_text,
                    on_b_click_fn=self._on_run_scenario_b_text,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

        # 카메라 타입 선택 프레임
        camera_type_frame = CollapsableFrame("Camera Type Selection", collapsed=False)
        
        with camera_type_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # 카메라 타입 선택
                with ui.HStack(spacing=5):
                    ui.Label("Selected Type:", width=80)
                    self._camera_type_model = ui.SimpleStringModel("basic")
                    self._camera_type_label = ui.Label("basic")
                
                # 카메라 타입 선택 버튼
                with ui.HStack(spacing=5):
                    self._basic_camera_btn = ui.Button("Basic Camera", clicked_fn=self._on_select_basic_camera)
                    self._drone_camera_btn = ui.Button("Drone Camera", clicked_fn=self._on_select_drone_camera)
        
        # 동적 UI 컨테이너
        self._dynamic_ui_container = ui.VStack(style=get_style(), spacing=5, height=0)
        
        # Basic Camera UI (초기에는 표시)
        with self._dynamic_ui_container:
            self._basic_camera_ui = ui.VStack(style=get_style(), spacing=5, height=0)
            with self._basic_camera_ui:
                # 카메라 이름 입력
                with ui.HStack(spacing=5):
                    ui.Label("Camera Name:", width=80)
                    self._basic_camera_name_model = ui.SimpleStringModel("basic_camera_1")
                    self._basic_camera_name_field = ui.StringField(self._basic_camera_name_model)
                
                # 카메라 위치 입력
                with ui.HStack(spacing=5):
                    ui.Label("Position X:", width=80)
                    self._basic_pos_x_model = ui.SimpleFloatModel(0.0)
                    self._basic_pos_x_field = ui.FloatField(self._basic_pos_x_model)
                
                with ui.HStack(spacing=5):
                    ui.Label("Position Y:", width=80)
                    self._basic_pos_y_model = ui.SimpleFloatModel(0.0)
                    self._basic_pos_y_field = ui.FloatField(self._basic_pos_y_model)
                
                with ui.HStack(spacing=5):
                    ui.Label("Position Z:", width=80)
                    self._basic_pos_z_model = ui.SimpleFloatModel(10.0)
                    self._basic_pos_z_field = ui.FloatField(self._basic_pos_z_model)
                
                # 카메라 회전 입력 (도 단위)
                with ui.HStack(spacing=5):
                    ui.Label("Rotation X (deg):", width=80)
                    self._basic_rot_x_model = ui.SimpleFloatModel(0.0)
                    self._basic_rot_x_field = ui.FloatField(self._basic_rot_x_model)
                
                with ui.HStack(spacing=5):
                    ui.Label("Rotation Y (deg):", width=80)
                    self._basic_rot_y_model = ui.SimpleFloatModel(0.0)
                    self._basic_rot_y_field = ui.FloatField(self._basic_rot_y_model)
                
                with ui.HStack(spacing=5):
                    ui.Label("Rotation Z (deg):", width=80)
                    self._basic_rot_z_model = ui.SimpleFloatModel(0.0)
                    self._basic_rot_z_field = ui.FloatField(self._basic_rot_z_model)
                
                # Basic 카메라 생성/제거 버튼
                with ui.HStack(spacing=5):
                    self._create_basic_camera_btn = ui.Button("Create Basic Camera", clicked_fn=self._on_create_basic_camera)
                    self._remove_basic_camera_btn = ui.Button("Remove Basic Camera", clicked_fn=self._on_remove_basic_camera)
            
            # Drone Camera UI (초기에는 숨김)
            self._drone_camera_ui = ui.VStack(style=get_style(), spacing=5, height=0)
            with self._drone_camera_ui:
                # 드론 카메라 이름 입력
                with ui.HStack(spacing=5):
                    ui.Label("Drone Name:", width=80)
                    self._drone_camera_name_model = ui.SimpleStringModel("drone_camera_1")
                    self._drone_camera_name_field = ui.StringField(self._drone_camera_name_model)
                
                # 드론 초기 위치 입력
                with ui.HStack(spacing=5):
                    ui.Label("Start Position X:", width=80)
                    self._drone_start_x_model = ui.SimpleFloatModel(0.0)
                    self._drone_start_x_field = ui.FloatField(self._drone_start_x_model)
                
                with ui.HStack(spacing=5):
                    ui.Label("Start Position Y:", width=80)
                    self._drone_start_y_model = ui.SimpleFloatModel(0.0)
                    self._drone_start_y_field = ui.FloatField(self._drone_start_y_model)
                
                with ui.HStack(spacing=5):
                    ui.Label("Start Position Z:", width=80)
                    self._drone_start_z_model = ui.SimpleFloatModel(10.0)
                    self._drone_start_z_field = ui.FloatField(self._drone_start_z_model)
                
                # 드론 카메라 생성/제거 버튼
                with ui.HStack(spacing=5):
                    self._create_drone_camera_btn = ui.Button("Create Drone Camera", clicked_fn=self._on_create_drone_camera)
                    self._remove_drone_camera_btn = ui.Button("Remove Drone Camera", clicked_fn=self._on_remove_drone_camera)
                
                # 체크포인트 관리 섹션
                ui.Separator()
                with ui.HStack(spacing=5):
                    ui.Label("Checkpoint Management", width=120)
                
                # 체크포인트 추가
                with ui.HStack(spacing=5):
                    ui.Label("Add Checkpoint:", width=100)
                    self._checkpoint_x_model = ui.SimpleFloatModel(10.0)
                    self._checkpoint_x_field = ui.FloatField(self._checkpoint_x_model)
                    self._checkpoint_y_model = ui.SimpleFloatModel(10.0)
                    self._checkpoint_y_field = ui.FloatField(self._checkpoint_y_model)
                    self._checkpoint_z_model = ui.SimpleFloatModel(10.0)
                    self._checkpoint_z_field = ui.FloatField(self._checkpoint_z_model)
                    self._add_checkpoint_btn = ui.Button("Add", clicked_fn=self._on_add_checkpoint)
                
                # 체크포인트 목록
                with ui.HStack(spacing=5):
                    ui.Label("Checkpoints:", width=80)
                
                # 체크포인트 리스트 컨테이너
                self._checkpoint_list_container = ui.VStack(style=get_style(), spacing=2, height=0)
                
                # 체크포인트 관리 버튼
                with ui.HStack(spacing=5):
                    self._clear_checkpoints_btn = ui.Button("Clear All Checkpoints", clicked_fn=self._on_clear_checkpoints)
                    self._update_checkpoint_list_btn = ui.Button("Update Checkpoint List", clicked_fn=self._update_checkpoint_list)
                
                # 드론 시뮬레이션 섹션
                ui.Separator()
                with ui.HStack(spacing=5):
                    ui.Label("Drone Simulation", width=120)
                
                # 시뮬레이션 속도 설정
                with ui.HStack(spacing=5):
                    ui.Label("Speed (m/s):", width=80)
                    self._drone_speed_model = ui.SimpleFloatModel(2.0)
                    self._drone_speed_field = ui.FloatField(self._drone_speed_model)
                
                # 시뮬레이션 컨트롤 버튼
                with ui.HStack(spacing=5):
                    self._start_drone_simulation_btn = ui.Button("Start Simulation", clicked_fn=self._on_start_drone_simulation)
                    self._stop_drone_simulation_btn = ui.Button("Stop Simulation", clicked_fn=self._on_stop_drone_simulation)
                
                # 드론 상태 표시
                with ui.HStack(spacing=5):
                    ui.Label("Drone Status:", width=80)
                    self._drone_status_label = ui.Label("Inactive")
        
        # 공통 카메라 목록 프레임
        camera_list_frame = CollapsableFrame("Camera Management", collapsed=False)
        
        with camera_list_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # 생성된 카메라 목록
                with ui.HStack(spacing=5):
                    ui.Label("All Cameras:", width=100)
                    self._camera_list_label = ui.Label("No cameras created")
                
                # 카메라 목록 업데이트 버튼
                self._update_camera_list_btn = ui.Button("Update Camera List", clicked_fn=self._update_camera_list)
        
        # 초기 UI 상태 설정 (Basic Camera UI 표시)
        self._show_basic_camera_ui()

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        self._articulation = None
        self._cuboid = None
        self._scenario = FrankaRmpFlowExampleScript()

    def _add_light_to_stage(self):
        """
        A new stage does not have a light by default.  This function creates a spherical light
        """
        sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(2)
        sphereLight.CreateIntensityAttr(100000)
        SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

    def _setup_scene(self):
        """
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.
        """
        create_new_stage()
        self._add_light_to_stage()

        loaded_objects = self._scenario.load_example_assets()

        # Add user-loaded objects to the World
        world = World.instance()
        for loaded_object in loaded_objects:
            world.scene.add(loaded_object)

    def _setup_scenario(self):
        """
        This function is attached to the Load Button as the setup_post_load_fn callback.
        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.
        """
        self._scenario.setup()

        # UI management
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True
        self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        self._scenario.reset()

        # UI management
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """This function is attached to the Run Scenario StateButton.
        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        This function will repeatedly advance the script in scenario.py until it is finished.

        Args:
            step (float): The dt of the current physics step
        """
        done = self._scenario.update(step)
        if done:
            self._scenario_state_btn.enabled = False

    def _on_run_scenario_a_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_a_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "RUN".

        This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
        the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
        through the left-hand UI toolbar.
        """
        self._timeline.play()

    def _on_run_scenario_b_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_b_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "STOP"

        Pausing the timeline on b_text is not strictly necessary for this example to run.
        Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
        the robot will stop getting new commands and the cube will stop updating without needing to
        pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
        forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
        this example prettier, but if curious, the user should observe what happens when this line is removed.
        """
        self._timeline.pause()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False
    
    # 카메라 관련 콜백 함수들
    def _on_create_basic_camera(self):
        """기본 카메라 생성 버튼 클릭 콜백"""
        camera_name = self._basic_camera_name_model.get_value_as_string()
        
        # 위치 값 가져오기
        pos_x = self._basic_pos_x_model.get_value_as_float()
        pos_y = self._basic_pos_y_model.get_value_as_float()
        pos_z = self._basic_pos_z_model.get_value_as_float()
        
        # 회전 값 가져오기 (도 단위)
        rot_x = self._basic_rot_x_model.get_value_as_float()
        rot_y = self._basic_rot_y_model.get_value_as_float()
        rot_z = self._basic_rot_z_model.get_value_as_float()
        
        from pxr import Gf
        import math
        
        # 위치와 회전 설정
        position = Gf.Vec3f(pos_x, pos_y, pos_z)
        
        # 도를 라디안으로 변환
        rot_x_rad = math.radians(rot_x)
        rot_y_rad = math.radians(rot_y)
        rot_z_rad = math.radians(rot_z)
        
        # 회전을 쿼터니언으로 변환 (XYZ 순서)
        from pxr import Gf
        rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), rot_x_rad) * \
                   Gf.Rotation(Gf.Vec3d(0, 1, 0), rot_y_rad) * \
                   Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_z_rad)
        
        success = self.camera_controller.create_basic_camera(camera_name, position, rotation)
        
        if success:
            self._update_camera_list()
            print(f"기본 카메라 '{camera_name}'이(가) 성공적으로 생성되었습니다. 위치: {position}, 회전: ({rot_x}°, {rot_y}°, {rot_z}°)")
        else:
            print(f"기본 카메라 생성에 실패했습니다.")
    
    def _on_remove_basic_camera(self):
        """기본 카메라 제거 버튼 클릭 콜백"""
        camera_name = self._basic_camera_name_model.get_value_as_string()
        self.camera_controller.remove_camera(camera_name)
        self._update_camera_list()
    
    def _on_create_drone_camera(self):
        """드론 카메라 생성 버튼 클릭 콜백"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        
        # 위치 값 가져오기
        pos_x = self._drone_start_x_model.get_value_as_float()
        pos_y = self._drone_start_y_model.get_value_as_float()
        pos_z = self._drone_start_z_model.get_value_as_float()
        
        from pxr import Gf
        position = Gf.Vec3f(pos_x, pos_y, pos_z)
        
        success = self.camera_controller.create_drone_camera(camera_name, position)
        
        if success:
            self._update_camera_list()
            print(f"드론 카메라 '{camera_name}'이(가) 성공적으로 생성되었습니다.")
        else:
            print(f"드론 카메라 생성에 실패했습니다.")
    
    def _on_remove_drone_camera(self):
        """드론 카메라 제거 버튼 클릭 콜백"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        self.camera_controller.remove_camera(camera_name)
        self._update_camera_list()
    
    def _on_add_checkpoint(self):
        """체크포인트 추가 버튼 클릭 콜백"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        
        # 체크포인트 위치 값 가져오기
        pos_x = self._checkpoint_x_model.get_value_as_float()
        pos_y = self._checkpoint_y_model.get_value_as_float()
        pos_z = self._checkpoint_z_model.get_value_as_float()
        
        from pxr import Gf
        position = Gf.Vec3f(pos_x, pos_y, pos_z)
        
        self.camera_controller.add_checkpoint(camera_name, position)
        self._update_checkpoint_list()
        
        # 입력 필드 초기화
        self._checkpoint_x_model.set_value(10.0)
        self._checkpoint_y_model.set_value(10.0)
        self._checkpoint_z_model.set_value(10.0)
    
    def _on_clear_checkpoints(self):
        """모든 체크포인트 제거 버튼 클릭 콜백"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        self.camera_controller.clear_checkpoints(camera_name)
        self._update_checkpoint_list()
    
    def _update_checkpoint_list(self):
        """체크포인트 목록 업데이트"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        checkpoints = self.camera_controller.get_checkpoints(camera_name)
        
        # 기존 체크포인트 리스트 UI 제거
        self._checkpoint_list_container.clear()
        
        if checkpoints:
            # 각 체크포인트에 대해 UI 생성
            for i, checkpoint in enumerate(checkpoints):
                with self._checkpoint_list_container:
                    with ui.HStack(spacing=5):
                        # 체크박스 (활성화/비활성화)
                        self._create_checkpoint_checkbox(camera_name, i)
                        
                        # 체크포인트 정보
                        checkpoint_text = f"CP {i+1}: ({checkpoint[0]:.1f}, {checkpoint[1]:.1f}, {checkpoint[2]:.1f})"
                        ui.Label(checkpoint_text, width=200)
                        
                        # 삭제 버튼
                        ui.Button(f"Remove", clicked_fn=lambda idx=i: self._on_remove_checkpoint(idx))
        else:
            # 체크포인트가 없을 때
            with self._checkpoint_list_container:
                ui.Label("No checkpoints", width=200)
    
    def _create_checkpoint_checkbox(self, camera_name: str, index: int):
        """체크포인트 체크박스 생성"""
        # 체크포인트 상태 가져오기 (기본값: 활성화)
        is_enabled = self.camera_controller.is_checkpoint_enabled(camera_name, index)
        
        # 체크박스 모델 생성
        checkbox_model = ui.SimpleBoolModel(is_enabled)
        
        # 체크박스 생성
        checkbox = ui.CheckBox(checkbox_model)
        
        # 체크박스 상태 변경 이벤트
        def on_checkbox_changed(model):
            enabled = model.get_value_as_bool()
            self.camera_controller.enable_checkpoint(camera_name, index, enabled)
            status = "활성화" if enabled else "비활성화"
            print(f"체크포인트 {index+1} {status}")
        
        checkbox_model.add_value_changed_fn(on_checkbox_changed)
    
    def _on_remove_checkpoint(self, index: int):
        """특정 체크포인트 제거"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        self.camera_controller.remove_checkpoint(camera_name, index)
        self._update_checkpoint_list()
    
    def _update_camera_list(self):
        """생성된 카메라 목록 업데이트"""
        camera_list = self.camera_controller.get_camera_list()
        if camera_list:
            camera_list_str = ", ".join(camera_list)
        else:
            camera_list_str = "No cameras created"
        
        self._camera_list_label.text = camera_list_str
    
    def _on_select_basic_camera(self):
        """기본 카메라 선택 버튼 콜백"""
        self._camera_type_model.set_value("basic")
        self._camera_type_label.text = "basic"
        
        # UI 동적 변경
        self._show_basic_camera_ui()
        
        print("기본 카메라가 선택되었습니다.")
    
    def _on_select_drone_camera(self):
        """드론 카메라 선택 버튼 콜백"""
        self._camera_type_model.set_value("drone")
        self._camera_type_label.text = "drone"
        
        # UI 동적 변경
        self._show_drone_camera_ui()
        
        print("드론 카메라가 선택되었습니다.")
    
    def _show_basic_camera_ui(self):
        """기본 카메라 UI 표시"""
        # Basic Camera UI만 표시
        if hasattr(self, '_basic_camera_ui') and hasattr(self, '_drone_camera_ui'):
            self._basic_camera_ui.visible = True
            self._drone_camera_ui.visible = False
    
    def _show_drone_camera_ui(self):
        """드론 카메라 UI 표시"""
        # Drone Camera UI만 표시
        if hasattr(self, '_basic_camera_ui') and hasattr(self, '_drone_camera_ui'):
            self._basic_camera_ui.visible = False
            self._drone_camera_ui.visible = True
    
    def _on_start_drone_simulation(self):
        """드론 시뮬레이션 시작 버튼 콜백"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        speed = self._drone_speed_model.get_value_as_float()
        
        # 활성화된 체크포인트만 가져오기
        enabled_checkpoints = self.camera_controller.get_enabled_checkpoints(camera_name)
        
        if not enabled_checkpoints:
            print("활성화된 체크포인트가 없습니다.")
            return
        
        # 시뮬레이션 시작
        success = self.drone_simulator.start_drone_simulation(camera_name, enabled_checkpoints, speed)
        
        if success:
            self._update_drone_status()
            print(f"드론 시뮬레이션이 시작되었습니다. 속도: {speed} m/s")
        else:
            print("드론 시뮬레이션 시작에 실패했습니다.")
    
    def _on_stop_drone_simulation(self):
        """드론 시뮬레이션 중지 버튼 콜백"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        self.drone_simulator.stop_drone_simulation(camera_name)
        self._update_drone_status()
        print("드론 시뮬레이션이 중지되었습니다.")
    
    def _update_drone_status(self):
        """드론 상태 업데이트"""
        camera_name = self._drone_camera_name_model.get_value_as_string()
        status = self.drone_simulator.get_drone_status(camera_name)
        
        if status["is_active"]:
            if status.get("is_moving", True):
                # 이동 중
                status_text = status.get("status_text", f"Active - CP {status['current_checkpoint']}/{status['total_checkpoints']} (Path: {status['current_path_point']}/{status['total_path_points']})")
            else:
                # 정지 상태
                status_text = status.get("status_text", f"Stopped - CP {status['current_checkpoint']}/{status['total_checkpoints']} (마지막 체크포인트 도달)")
        else:
            status_text = "Inactive"
        
        self._drone_status_label.text = status_text
