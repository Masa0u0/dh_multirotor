# dh_multirotor

## 実行例

---

```bash
$ roslaunch rotors_gazebo mav.launch mav_name:=iris  # TODO: multirotor_gazeboが整ったら削除
$ roslaunch multirotor_gazebo gazebo.launch  # TODO: ここでシミュレータを立ち上げる
$ roslaunch multirotor_controller controller.launch
$ roslaunch multirotor_keyboard_teleop keyboard_teleop.launch
$ rosrun plotjuggler plotjuggler
```
