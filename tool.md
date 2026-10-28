# 编译调试快捷工具
gdb ./percent            # 进入 gdb 提示符 (gdb)

(head -11 temp1.pcd && tail -n +12 temp1.pcd | head -100) > front100.pcd

./percent > log.txt 2>&1

cmake ../src                # CMakeLists.txt 在这里