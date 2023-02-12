'''
Pass in the argument 'pre-run' to not trigger the run of the test, but rather evaluate the existing run.log file.
'''

import subprocess
import sys

if __name__ == '__main__':
    cli_arg = sys.argv[1] if len(sys.argv) >= 2 else None

    if cli_arg != 'pre-run':
        subprocess.run('cargo test --features debug_assertions -- test_2_bodies > run.log',
                       shell=True)

    with open('run.log') as f:
        lines = f.readlines()
        lines = (line.split('|', 1) for line in lines)

        lines = [line[1].strip() for line in lines if len(line) == 2]

        setup = [line for line in lines if line.startswith('SETUP')]

        # SETUP line is of the form: key=value,key=value,key=value
        # where the keys are: r0, r1, rho, init_impact_v, sep_dis, dt, steps, desired_steps
        match setup:
            case []:
                raise Exception("No setup line found.")
            case [s]:
                s = s[5:].strip()
                setup_ops = dict(op.strip().split('=', maxsplit=1)
                                 for op in s.split(','))
            case _:
                raise Exception("More than one setup line found.")

        lines = (line for line in lines if line.startswith(
            'STEP') or line.startswith('SUB_STEP'))

        steps = []
        '''
        0 self.current_time,
        1 p_idx,
        2 p.p[0],
        3 p.v[0]
        '''

        sub_steps = []
        '''
        0 current_time,
        1 p1i.0,
        2 p1.p[0],
        3 p1.v[0],
        4 p2i.0,
        5 p2.p[0],
        6 p2.v[0]
        '''

        for line in lines:
            if line.startswith('STEP'):
                line = line[5:]
                steps.append(tuple(float(e) for e in line.split(',')))
            elif line.startswith('SUB_STEP'):
                line = line[9:]
                sub_steps.append(tuple(float(e) for e in line.split(',')))
            else:
                assert False

    # analysis here?
    def collect_for_particle(n: int) -> list[tuple[float, float]]:
        '''
        time, x
        '''
        particle_x = []
        for s in steps:
            if s[1] == n:
                particle_x.append((s[0], s[2]))
        for s in sub_steps:
            if s[1] == n:
                particle_x.append((s[0], s[2]))
            elif s[4] == n:
                particle_x.append((s[0], s[5]))

        particle_x.sort(key=lambda x: x[0])
        return particle_x

    r0 = float(setup_ops['r0'])
    r1 = float(setup_ops['r1'])

    p0_t, p0_x = list(zip(*collect_for_particle(0)))
    p1_t, p1_x = list(zip(*collect_for_particle(1)))

    import matplotlib.pyplot as plt
    plt.scatter(p0_t, p0_x)
    plt.scatter(p1_t, p1_x)

    plt.scatter(p0_t, [x + r0 for x in p0_x], marker='x')
    plt.scatter(p0_t, [x - r0 for x in p0_x], marker='x')

    plt.scatter(p1_t, [x + r1 for x in p1_x], marker='x')
    plt.scatter(p1_t, [x - r1 for x in p1_x], marker='x')
    plt.show()

    print(f'sub_steps={len(sub_steps)}')
