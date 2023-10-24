import random
from random import randint 

test_file = open('test.txt', 'w')

N = randint(1, 10000)
test_file.write(str(N))
test_file.write('\n')

for i in range(N):
    for j in range(3):
        test_file.write(str(random.uniform(-1000, 1000)))
        test_file.write('\n')
        test_file.write(str(random.uniform(-1000, 1000)))
        test_file.write('\n')
        test_file.write(str(random.uniform(-1000, 1000)))
        test_file.write('\n')


