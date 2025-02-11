# vis_launch.launch
> 기존의 시뮬레이션(bag 파일을 활용한 시뮬레이션, GUI 없음)
* `pubsubcp.py` ✅
  * `bounding_box_design.py`로 대체
* `tf_broadcaster.py` ✅
* `test_22.bag` ✅
* `bbox_subscriber.py`
  * `robot_manipulation.py`로 대체
  * Raspi에서 실행
* `dofbot.rviz` ✅

# control_sim.launch
> 임의 객체를 생성 및 트래킹하는 시뮬레이션 (GUI)
* `generate_random_objs.py` ✅
* `generated_objs_gui.py`
* `generated_objs_rviz.py`
* `generated_objs_real.py`
  * Raspi에서 실행
* `tf_broadcaster_new.py`
* `generated_objs.rviz`

# bag_sim.launch
> bag 파일을 활용한 시뮬레이션 (GUI)
* `bounding_box_design.py` ✅
* `bbox_gui.py`
* `bbox_rviz.py`
* `bbox_real.py`
  * Raspi에서 실행
* `tf_broadcaster_new.py`
* `bbox.rviz`
* `test_24.bag`