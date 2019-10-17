import math
import numpy as np

def cross_product(vector_one, vector_two):
    result_x = vector_one[1] * vector_two[2] - vector_one[2] * vector_two[1]
    result_y = vector_one[2] * vector_two[0] - vector_one[0] * vector_two[2]
    result_z = vector_one[1] * vector_two[1] - vector_one[1] * vector_two[0]

    result = [result_x, result_y, result_z]

    return result


def scalar_product(vector_one, vector_two):
    return (vector_one[0] * vector_two[0]) + (vector_one[1] * vector_two[1]) + (vector_one[2] * vector_two[2])


def distance_between_vector(vector_one, vector_two):
    return math.sqrt(pow(vector_one[0] - vector_two[0], 2) +
                     pow(vector_one[1] - vector_two[1], 2) +
                     pow(vector_one[2] - vector_two[2]))


def modulus(vector):
    print("vector", vector)
    return np.sqrt(np.power(vector[0], 2) + np.power(vector[1], 2) + np.power(vector[2], 2))
