import numpy as np
import json

COLOR_DATA_DIR = "color_data/"

# Dictionary mapping color names to their respective CSV files
COLOR_DATA = {
    "white": COLOR_DATA_DIR+"white_data.csv",   # Hallway
    "purple": COLOR_DATA_DIR+"purple_data.csv",    # Burning room
    "yellow": COLOR_DATA_DIR+"yellow_data.csv",    # Room to avoid
    "green": COLOR_DATA_DIR+"green_data.csv",     # Green Card
    "red": COLOR_DATA_DIR+"red_data.csv",     # Red Card
    "orange": COLOR_DATA_DIR+"crange_data.csv"     # Entrance line
}


def multivariate_gaussian(filename):
    """
    Computes the mean vector and covariance matrix for RGB values stored in a CSV file.
    
    Parameters:
        filename (str): Path to the CSV file containing RGB values.
            or
        filename (list): list of lists of rgb values
    
    Returns:
        tuple: (mean vector, covariance matrix)
    """
    if type(filename) == str:
        rgb_values = np.genfromtxt(filename, delimiter=',')
    elif type(filename) == list:
        rgb_values = np.array(filename)
    else:
        raise Exception ("multivatiate_gaussian() received bad input: \nneeds to be either str (filename) or list")
    
    rgb_values /= 255.0  # Normalize RGB values to [0,1] range
    mu = np.mean(rgb_values, axis=0)  # Compute mean vector
    sigma = np.cov(rgb_values, rowvar=False)  # Compute covariance matrix
    #print("GAUSSIAN MATRIX READ: ", rgb_values, '\n')
    return mu, sigma


def save_mult_gaussian(color_name, mu, sigma):
    """
    Saves the mean vector and covariance matrix of a color to a JSON file.
    
    Parameters:
        color_name (str): Name of the color.
        mu (np.array): Mean vector.
        sigma (np.array): Covariance matrix.
    """
    data = {
        "color": color_name,
        "mean": mu.tolist(),
        "covariance": sigma.tolist()
    }
    
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
        tuple: (mean vector as np.array, covariance matrix as np.array)
    """
    filepath = "color_mult_gaussian/" + color_name + ".json"
    
    with open(filepath, "r") as file:
        try:
            data = json.load(file)
            if data["color"] == color_name:
                return np.array(data["mean"]), np.array(data["covariance"])
        except json.JSONDecodeError as e:
            print("Error loading JSON:", e)
            return None, None
    
    raise ValueError(f"Color '{color_name}' not found in {filepath}")


def test_retreive():
    """
    Tests retrieval of a stored mean vector and covariance matrix against computed values.
    """
    og_mu, og_sig = multivariate_gaussian(COLOR_DATA_DIR+"red_data.csv")
    print("OG MU: ", og_mu)
    print("OG SIGMA: ", og_sig, '\n')

    av, sig = retreive_mult_gaussian("red")
    print("GOT MU: ", av)
    print("GOT SIGMA: ", sig, '\n')

    assert (og_mu == av).all()
    assert (og_sig == sig).all()


def bhattacharyya_distance(mu_u, sigma_u, mu_i, sigma_i):
    """
    Compute the Bhattacharyya distance between two multivariate Gaussians.
    
    Parameters:
        mu_u (np.array): Mean vector of the unknown color (3,)
        sigma_u (np.array): Covariance matrix of the unknown color (3x3)
        mu_i (np.array): Mean vector of the known color (3,)
        sigma_i (np.array): Covariance matrix of the known color (3x3)
    
    Returns:
        float: Bhattacharyya distance
    """
    sigma = (sigma_i + sigma_u) / 2
    mean_diff = mu_u - mu_i

    try:
        sigma_inv = np.linalg.inv(sigma)
    except np.linalg.LinAlgError:
        sigma += np.eye(3) * 1e-6  # Add small value to diagonal if singular
        sigma_inv = np.linalg.inv(sigma)
    
    det_sigma = np.linalg.det(sigma)
    det_sigma_i = np.linalg.det(sigma_i)
    det_sigma_u = np.linalg.det(sigma_u)
    
    if det_sigma_u == 0 or det_sigma_i == 0 or det_sigma == 0:
        det_sigma_u += 1e-6
        det_sigma_i += 1e-6
        det_sigma += 1e-6

    left_term = (1/8) * mean_diff.T @ sigma_inv @ mean_diff
    right_term = 0.5 * np.log(det_sigma / np.sqrt(det_sigma_i * det_sigma_u))
    
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
    
    return min(bhat_distances, key=bhat_distances.get)  # Return color with smallest distance


def write_unknown_color(color, nb_data_points):
    """
    Writes a sample of the given color's RGB data to a new file for testing purposes.
    """
    with open(COLOR_DATA_DIR+f"/{color}_data.csv", 'r') as coloF:
        with open("unknown_color.csv", 'w') as uF:
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
