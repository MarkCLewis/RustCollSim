import json
import matplotlib.pyplot as plt

'''
one state snapshot (example):
{
    "E": 0.00000000000000000000000000000012641388329940234,
    "GPE": -0.0000000000000000000000000000000026012923004290486,
    "KE": 0.00000000000000000000000000000012901517559983138, 
    "states": [
        {
            "displacement": {
                "x": -0.00000019995799995342756,
                "y": 0.0,
                "z": 0.0
            },
            "velocity": {
                "x": 0.0000020000042341393583,
                "y": 0.0,
                "z": 0.0
            }
        },
        {
            "displacement": {
                "x": 0.00000019995799995342756,
                "y": 0.0,
                "z": 0.0
            },
            "velocity": {
                "x": -0.0000020000042341393583,
                "y": 0.0,
                "z": 0.0
            }
        }
    ],
    "time": 0.000020000000000000005
}
'''

def main(obj):
    drag = obj['drag']
    k = obj['k']

    masses = obj['masses']
    sizes = obj['sizes']

    states = obj['states']

    
    selector = 'EvsTime'


    if selector == 'EvsTime':
        x = [s['time'] for s in states]
        y = [s['E'] for s in states]

        plt.plot(x, y)

        yName = 'Energy'
        xName = 'Time'

        plt.xlabel(xName)
        plt.ylabel(yName)
        plt.title(f'{yName} vs. {xName}')
        #plt.show()

        plt.savefig('test.png')



if __name__ == '__main__':
    with open('../out.json') as f:
        obj = json.load(f)
    main(obj)