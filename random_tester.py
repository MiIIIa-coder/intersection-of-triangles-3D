import random
from random import randint 

test_file = open('test.txt', 'w')

N = randint(80000, 100000)
test_file.write(str(N))
test_file.write('\n')

for i in range(N):
    for j in range(3):
        test_file.write(str(random.uniform(-10000, 10000)))
        test_file.write('\n')
        test_file.write(str(random.uniform(-10000, 10000)))
        test_file.write('\n')
        test_file.write(str(random.uniform(-10000, 10000)))
        test_file.write('\n')


