import subprocess
import os


def mesh_fix(inputmesh, outputmesh='default', join_small_parts=False, overwrite=True, print_info=True):
    '''
    MeshFix
    ref:https://github.com/MarcoAttene/MeshFix-V2.1
    :param inputmesh: STL, OFF, PLY file path.
    :param outputmesh: file name
    :param join_small_parts: force the software to join the connected components whose boundary loops are closer to each other. Otherwise, only the largest component is kept while the others are considered as "noise".
    :param output_stl: ouoput file format, true, stl; false, off
    :param overwrite: overwrite existing output file
    :return: nothing
    '''
    # check MeshFix exist
    current_path = os.path.abspath(__file__)
    dir_path = os.path.dirname(current_path)
    MeshFix_path = os.path.join(dir_path, "../bin/MeshFix")

    if not os.path.exists(MeshFix_path):
        print("MeshFix do not exist! Run 'install.sh' to fix this problem.")
        return False

    if outputmesh=='default':
        outputmesh = inputmesh.split('.')[0]+'_fixed.'+inputmesh.split('.')[1]
        print(outputmesh)

    commands = [MeshFix_path, inputmesh, outputmesh]

    if join_small_parts:
        commands += '-a'
    if not overwrite:
        commands += '-x'

    try:
        process = subprocess.Popen(commands, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # 向进程写入输入
        process.stdin.write(b"\n")
        process.stdin.flush()

        # 等待进程结束并获取输出
        stdout, stderr = process.communicate()

        # 如果MeshFix正确执行并返回结果，将stdout打印在控制台
        if stdout and print_info:
            print("STDOUT:\n", stdout.decode())

        # 如果MeshFix在执行过程中产生错误，将stderr打印在控制台
        if stderr and print_info:
            print("STDERR:\n", stderr.decode())

    except Exception as e:
        if print_info:
            print(f"Error occurred: {e}")
