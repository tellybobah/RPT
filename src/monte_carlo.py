import math
import utils as utils
import numpy as np

"""
"""


def find_h_rho(position_particle, position_detector, orientation_detector):
    # position detector
    distance_vector = [position_detector[0] - position_particle[0],
                      position_detector[1] - position_particle[1],
                      position_detector[2] - position_particle[2]]

    # distance
    distance = utils.cross_product(distance_vector, orientation_detector)

    # height
    height = utils.modulus(distance)

    # rho

    rho = np.fabs(utils.scalar_product(distance_vector, orientation_detector))

    print("print height and rho",height,rho)
    return height, rho


"""
"""


def depht_two(tetha_critical, theta, alpha, rho, height, lenght, ray):
    # lambda
    lamda = math.sqrt(pow(((height + lenght) * math.tan(theta)), 2) + pow(rho, 2) - 2 * (height + lenght) * math.tan(
        theta) * rho * math.cos(alpha))

    detph = 0

    if lamda <= ray:
        detph = lenght / math.cos(theta)
    else:
        detph = (-height / math.cos(theta)) + (rho * math.cos(alpha) + math.sqrt(pow(ray, 2) - pow((rho) * math.sin(alpha)), 2))

    return detph

    """
    """

def depht_one(tetcri, theta, alpha, rho, height, oa_segment, ob_segment, length):

    depth = 0

    if theta < tetcri:
        if oa_segment / math.tan(theta) >= (height + length):
            depth = -ob_segment / math.sin(theta) + (height + length) / math.cos(theta)
        else:
            depth = (oa_segment - ob_segment) / math.sin(theta)
    else:
        if oa_segment / math.tan(theta) >= (height + length):
            depth = length / math.cos(theta)
        else:
            depth = oa_segment / math.sin(theta) - height / math.cos(theta)

    return depth

    """
    """

def ran(idum, ir):
    m = 714025
    ia = 1366
    ic = 150889
    iff = 0
    rm = 1 / m
    rtemp =0

    if (idum < 0) or (iff == 0):
        iff = 1
        idum = math.fabs(math.fmod((ic - idum), m))

        for j in range(97):
            idum = math.fmod(ia * idum + ic, m)
            ir[j] = idum

        idum = math.fmod(ia * idum + ic, m)

        iy = idum

    k = 1 + (96 * iy) / m
    j = math.floor(k)

    if (j > 96) or (j < 0):
        j = 0

        iy = ir[j]

        rtemp = iy * rm

        idum = math.fmod(ia * idum + ic, m)

    return (rtemp, idum, iff)

    """

    """

def simulate_ray(position_particle, position_detector, orientation_detector, ss, tau, wallmu,
                     ray, length):

    dic = {}
    idum = -1
    tt = 5
    gperd = 2
    perech = 0.09
    solang = 0
    inteff = 0
    solangsqr = 0
    munai = 0.055
    # getting the height and the row compared to the current location

    height, rho = find_h_rho(position_particle, position_detector, orientation_detector)
    photopeak = 0.4
    mu = 3

    if rho > ray:
        alpha_max = math.asin(ray / rho)
        walf = alpha_max

        for silumation in range(12000):
            ran3 = ran(idum, dic)
            random_ray = ran3[0]
            idum = ran3[1]

            alpha = alpha_max * (2 * random_ray - 1)
            oa_segment = rho * math.cos(alpha) + math.sqrt(pow(ray, 2) - pow((rho * math.sin(alpha)), 2))
            tetmax = math.atan(oa_segment / height)
            ob_segment = rho * math.cos(alpha) - math.sqrt(pow(ray, 2) - pow((rho * math.sin(alpha)), 2))
            theta_min = math.atan(ob_segment / (height + length))
            theta_cri = math.atan(ob_segment / height)

            ran4 = ran(idum, dic)
            random_ray = ran4[0]
            idum = ran4[1]

            theta = math.acos(math.cos(theta_min) - random_ray * (math.cos(theta_min) - math.cos(tetmax)))
            wthet = 0.5 * (math.cos(theta_cri) - math.cos(tetmax))

            d_one = depht_one(theta_cri, theta, alpha, rho, height, oa_segment, ob_segment, length)

            prob = 1 - (math.exp(-mu * d_one))

            prob = prob * math.exp(-wallmu * tt / math.cos(theta))

            solang = solang + wthet * walf
            effint = prob * wthet * walf
            inteff = inteff + effint
            solangsqr = solangsqr + pow((wthet * walf), 2)

        solang1 = solang / 12000
        inteff = inteff / 12000
        term = gperd * ((3.7E10) * (ss)) * photopeak * inteff
        count = perech * term / (1 + (tau) * term)
        return count

    else :
        tetmax = math.atan((ray + rho) / height)
        tetcri = math.atan((ray - rho) / height)
        tetmin = 0

        wthet = 0.5 * (math.cos(tetmin) - math.cos(tetmax))
        for p in range(1200):

            ran5 = ran(idum, dic)
            r = ran5[0]
            idum = ran5[1]

            theta = math.acos(math.cos(tetmin) - r * (math.cos(tetmin) - math.cos(tetmax)));
            if theta >= tetcri :
                alfmax = math.acos((pow(rho, 2) + pow((height * math.tan(theta)), 2) - pow(ray, 2)) / (2 * height * rho * math.tan(theta)));
                walf = alfmax / math.pi
            else :

                alfmax = math.pi
                walf = 1

            ran6=ran(idum, dic)
            r=ran6[0]
            idum=ran6[1]

            alpha = alfmax * (2 * r - 1)

            d2 = depht_two(tetcri, theta, alpha, rho, height, length, ray)
            prob = 1 - math.exp(-munai * d2)
            prob = prob * math.exp((-(wallmu) * tt) / (math.cos(theta)))
            print(d2)
            solang = solang + wthet * walf
            effint = prob * wthet * walf
            inteff = inteff + wthet * walf * prob
            solangsqr = solangsqr + pow((wthet * walf), 2)

        solang1 = solang / 12000
        inteff = inteff / 12000
        term = gperd * ((3.7E10) * (ss)) * photopeak * inteff
        count = perech * term / (1 + (tau) * term)
        print("perech ", perech, "term", term,"tau", tau, "term", term)
        return count


POSITION_X = +6.2412245E-002
POSITION_Y = -2.1650000E-001
POSITION_Z = -1.4630561E-001
ORIENTATION_X = +5.0683278E-001
ORIENTATION_Y = -6.4499440E-001
ORIENTATION_Z = +5.0683278E-001
orientation_detector = [ORIENTATION_X,ORIENTATION_Y,ORIENTATION_Z]
position_detector = [POSITION_X,POSITION_Y,POSITION_Z]
print(simulate_ray([11.117,9.463,52.5],orientation_detector,orientation_detector,4.440573,5.17422e-06,0.55,36,72))