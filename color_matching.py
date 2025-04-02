import json
import csv
import math

# Attemting to replace the np dependency with our own functions

COLOR_DATA_DIR = "color_data/"

# Dictionary mapping color names to their respective CSV files
COLOR_DATA = {
    "white": COLOR_DATA_DIR + "white_data.csv",  # Hallway
    "purple": COLOR_DATA_DIR + "purple_data.csv",  # Burning room
    "yellow": COLOR_DATA_DIR + "yellow_data.csv",  # Room to avoid
    "green": COLOR_DATA_DIR + "green_data.csv",  # Green Card
    "red": COLOR_DATA_DIR + "red_data.csv",  # Red Card
    "orange": COLOR_DATA_DIR + "crange_data.csv",  # Entrance line
}


def read_csv_to_list(filename):
    """
    Reads RGB values from a CSV file into a list of lists.

    Parameters:
        filename (str): Path to the CSV file containing RGB values.

    Returns:
        list: A list of lists containing RGB values as floats.
    """
    rgb_values = []
    with open(filename, "r") as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            # Convert string values to float
            rgb_values.append([float(val) for val in row])
    return rgb_values


def compute_mean(data):
    """
    Computes the mean vector for a list of vectors.

    Parameters:
        data (list): List of vectors (each vector is a list of numbers).

    Returns:
        list: Mean vector.
    """
    if not data:
        return []

    num_features = len(data[0])
    mean = [0] * num_features

    for vector in data:
        for i in range(num_features):
            mean[i] += vector[i]

    return [val / len(data) for val in mean]


def compute_covariance(data, mean):
    """
    Computes the covariance matrix for a list of vectors.

    Parameters:
        data (list): List of vectors (each vector is a list of numbers).
        mean (list): Mean vector for the data.

    Returns:
        list: Covariance matrix as a list of lists.
    """
    n = len(data)
    if n <= 1:
        return [[0] * len(mean) for _ in range(len(mean))]

    dim = len(mean)
    cov = [[0] * dim for _ in range(dim)]

    for vector in data:
        for i in range(dim):
            for j in range(dim):
                cov[i][j] += (vector[i] - mean[i]) * (vector[j] - mean[j])

    for i in range(dim):
        for j in range(dim):
            cov[i][j] /= n - 1

    return cov


def multivariate_gaussian(filename):
    """
    Computes the mean vector and covariance matrix for RGB values.

    Parameters:
        filename (str): Path to the CSV file containing RGB values.
            or
        filename (list): list of lists of rgb values

    Returns:
        tuple: (mean vector, covariance matrix)
    """
    if isinstance(filename, str):
        rgb_values = read_csv_to_list(filename)
    elif isinstance(filename, list):
        rgb_values = filename
    else:
        raise Exception(
            "multivatiate_gaussian() received bad input: \nneeds to be either str (filename) or list"
        )

    # Normalize RGB values to [0,1] range
    rgb_values = [[val / 255.0 for val in row] for row in rgb_values]

    mu = compute_mean(rgb_values)  # Compute mean vector
    sigma = compute_covariance(rgb_values, mu)  # Compute covariance matrix

    return mu, sigma


def save_mult_gaussian(color_name, mu, sigma):
    """
    Saves the mean vector and covariance matrix of a color to a JSON file.

    Parameters:
        color_name (str): Name of the color.
        mu (list): Mean vector.
        sigma (list): Covariance matrix.
    """
    data = {"color": color_name, "mean": mu, "covariance": sigma}

    filepath = "color_mult_gaussian/" + color_name + ".json"
    with open(filepath, "w") as file:
        json.dump(data, file, indent=4)


def save_mult_gaussian_for_all_colors():
    """
    Computes and saves the mean vector and covariance matrix for all colors in COLOR_DATA.
    """
    for color in COLOR_DATA:
        mu, sigma = multivariate_gaussian(COLOR_DATA[color])
        save_mult_gaussian(color, mu, sigma)


def retreive_mult_gaussian(color_name):
    """
    Retrieves the mean vector and covariance matrix of a color from a JSON file.

    Parameters:
        color_name (str): Name of the color.

    Returns:
        tuple: (mean vector as list, covariance matrix as list of lists)
    """
    filepath = "color_mult_gaussian/" + color_name + ".json"

    with open(filepath, "r") as file:
        try:
            data = json.load(file)
            if data["color"] == color_name:
                return data["mean"], data["covariance"]
        except json.JSONDecodeError as e:
            print("Error loading JSON:", e)
            return None, None

    raise ValueError(f"Color '{color_name}' not found in {filepath}")


def test_retreive():
    """
    Tests retrieval of a stored mean vector and covariance matrix against computed values.
    """
    og_mu, og_sig = multivariate_gaussian(COLOR_DATA_DIR + "red_data.csv")
    print("OG MU: ", og_mu)
    print("OG SIGMA: ", og_sig, "\n")

    av, sig = retreive_mult_gaussian("red")
    print("GOT MU: ", av)
    print("GOT SIGMA: ", sig, "\n")

    # Check if values are equal (within a small epsilon for floating-point comparison)
    eps = 1e-6
    mean_equal = all(abs(m1 - m2) < eps for m1, m2 in zip(og_mu, av))

    sigma_equal = True
    for row1, row2 in zip(og_sig, sig):
        if not all(abs(s1 - s2) < eps for s1, s2 in zip(row1, row2)):
            sigma_equal = False
            break

    assert mean_equal
    assert sigma_equal


def matrix_determinant(matrix):
    """
    Compute the determinant of a 3x3 matrix.
    """
    if len(matrix) != 3 or len(matrix[0]) != 3:
        raise ValueError("Only 3x3 matrices are supported")

    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]

    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def matrix_inverse(matrix):
    """
    Compute the inverse of a 3x3 matrix.
    """
    if len(matrix) != 3 or len(matrix[0]) != 3:
        raise ValueError("Only 3x3 matrices are supported")

    det = matrix_determinant(matrix)
    if abs(det) < 1e-6:  # Close to singular
        # Add small value to diagonal
        for i in range(3):
            matrix[i][i] += 1e-6
        det = matrix_determinant(matrix)

    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]

    # Calculate cofactor matrix
    cofactor = [
        [(e * i - f * h), -(d * i - f * g), (d * h - e * g)],
        [-(b * i - c * h), (a * i - c * g), -(a * h - b * g)],
        [(b * f - c * e), -(a * f - c * d), (a * e - b * d)],
    ]

    # Transpose and divide by determinant
    inverse = [[cofactor[j][i] / det for j in range(3)] for i in range(3)]
    return inverse


def matrix_add(A, B):
    """
    Add two matrices element-wise.
    """
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def matrix_scalar_multiply(matrix, scalar):
    """
    Multiply a matrix by a scalar.
    """
    return [
        [matrix[i][j] * scalar for j in range(len(matrix[0]))]
        for i in range(len(matrix))
    ]


def vector_subtract(v1, v2):
    """
    Subtract v2 from v1 element-wise.
    """
    return [v1[i] - v2[i] for i in range(len(v1))]


def matrix_vector_multiply(matrix, vector):
    """
    Multiply a matrix by a vector.
    """
    return [
        sum(matrix[i][j] * vector[j] for j in range(len(vector)))
        for i in range(len(matrix))
    ]


def vector_dot_product(v1, v2):
    """
    Compute the dot product of two vectors.
    """
    return sum(v1[i] * v2[i] for i in range(len(v1)))


def bhattacharyya_distance(mu_u, sigma_u, mu_i, sigma_i):
    """
    Compute the Bhattacharyya distance between two multivariate Gaussians.

    Parameters:
        mu_u (list): Mean vector of the unknown color
        sigma_u (list): Covariance matrix of the unknown color
        mu_i (list): Mean vector of the known color
        sigma_i (list): Covariance matrix of the known color

    Returns:
        float: Bhattacharyya distance
    """
    # Compute (sigma_i + sigma_u) / 2
    sigma = matrix_scalar_multiply(matrix_add(sigma_i, sigma_u), 0.5)
    mean_diff = vector_subtract(mu_u, mu_i)

    try:
        sigma_inv = matrix_inverse(sigma)
    except Exception:
        # Add small value to diagonal if singular
        for i in range(3):
            sigma[i][i] += 1e-6
        sigma_inv = matrix_inverse(sigma)

    # First term: (1/8) * mean_diff.T @ sigma_inv @ mean_diff
    temp = matrix_vector_multiply(sigma_inv, mean_diff)
    left_term = vector_dot_product(mean_diff, temp) / 8.0

    # Calculate determinants
    det_sigma = matrix_determinant(sigma)
    det_sigma_i = matrix_determinant(sigma_i)
    det_sigma_u = matrix_determinant(sigma_u)

    # Handle determinants close to zero
    if abs(det_sigma_u) < 1e-6 or abs(det_sigma_i) < 1e-6 or abs(det_sigma) < 1e-6:
        det_sigma_u = max(det_sigma_u, 1e-6)
        det_sigma_i = max(det_sigma_i, 1e-6)
        det_sigma = max(det_sigma, 1e-6)

    # Second term: 0.5 * log(det_sigma / sqrt(det_sigma_i * det_sigma_u))
    right_term = 0.5 * math.log(det_sigma / math.sqrt(det_sigma_i * det_sigma_u))

    return left_term + right_term


def match_unknown_color(filename):
    """
    Matches an unknown color against known colors using Bhattacharyya distance.

    Parameters:
        filename (str): CSV file containing RGB values of the unknown color.
            or
        filename (list): list of lists of rgb values

    Returns:
        str: The closest matching color.
    """
    mu_u, sigma_u = multivariate_gaussian(filename)

    bhat_distances = {}
    for color in COLOR_DATA.keys():
        mu_i, sigma_i = retreive_mult_gaussian(color)
        bhat_distances[color] = bhattacharyya_distance(mu_u, sigma_u, mu_i, sigma_i)

    return min(
        bhat_distances, key=bhat_distances.get
    )  # Return color with smallest distance


def write_unknown_color(color, nb_data_points):
    """
    Writes a sample of the given color's RGB data to a new file for testing purposes.
    """
    with open(COLOR_DATA_DIR + f"/{color}_data.csv", "r") as coloF:
        with open("unknown_color.csv", "w") as uF:
            for i, line in enumerate(coloF):
                if i >= nb_data_points:
                    break
                uF.write(line)


def test_matching_color():
    """
    Tests the color matching system with different known colors.
    """
    for color in ["red", "orange", "white"]:
        write_unknown_color(color, 20)
        given_color = match_unknown_color("unknown_color.csv")
        if given_color == color:
            print(f"{color} test Passed\n")
        else:
            print(f"{color} test Failed\n")


if __name__ == "__main__":
    test_matching_color()

