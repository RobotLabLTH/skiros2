import skiros2_task.core.pddl_interface as pddl

def main():
    print(pddl.PddlInterface().invokePlanner(False))

if __name__ == '__main__':
    main()
