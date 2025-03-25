import numpy as np
import json

COLOR_DATA = {
    "white": "color_data/white_data.csv",   # Hallway
    "purple": "color_data/purple_data.csv",    # Burning room
    "yellow": "color_data/yellow_data.csv",    # Room to avoid
    "green": "color_data/green_data.csv",     # Green Card
    "red": "color_data/red_data.csv",     # Red Card
    #"black": "black_data.csv",         # Grid lines
    "orange": "color_data/orange_data.csv"     # Entrance line
}


def multivariate_gaussian(filename):
    rgb_values = np.genfromtxt(filename, delimiter=',')
    
    rgb_values /= 255.0     # normalize

    mu = np.mean(rgb_values, axis=0)  # mean of each column

    sigma = np.cov(rgb_values, rowvar=False)  # covariance matrix

    return mu, sigma


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

    # Compute the inverse of the covariance matrix
    try:
        sigma_inv = np.linalg.inv(sigma)
    except np.linalg.LinAlgError:
        # If the matrix is singular, add a small value to the diagonal and retry
        sigma += np.eye(3) * 1e-6
        sigma_inv = np.linalg.inv(sigma)
    
    # Determinants
    det_sigma = np.linalg.det(sigma)
    det_sigma_i = np.linalg.det(sigma_i)
    det_sigma_u = np.linalg.det(sigma_u)
    # Ensure determinants are not zero
    if det_sigma_u == 0 or det_sigma_i == 0 or det_sigma == 0:
        det_sigma_u += 1e-6
        det_sigma_i += 1e-6
        det_sigma += 1e-6


    left_term = (1/8) * mean_diff.T @ sigma_inv @ mean_diff
    right_term = 0.5 * np.log(det_sigma / np.sqrt(det_sigma_i * det_sigma_u))

    return left_term + right_term



def save_mult_gaussian(color_name, mu, sigma):
    data = {
        "color": color_name,
        "mean": mu.tolist(),
        "covariance": sigma.tolist()
    }

    filepath = "color_mult_gaussian/" + color_name +".json"
    with open(filepath, "w") as file:
        json.dump(data, file, indent=4)

    

def save_mult_gaussian_for_all_colors():
    "writes statistics for all colors in COLOR_DATA using color_statistics()"
    for color in COLOR_DATA:
        mu, sigma = multivariate_gaussian(COLOR_DATA[color])
        save_mult_gaussian(color, mu, sigma)



def retreive_mult_gaussian(color_name):
    filepath = "color_mult_gaussian/" + color_name +".json"

    with open(filepath, "r") as file:
        try:
            data = json.load(file)  # Load full JSON file (if stored as an array)
            if isinstance(data, list):
                for entry in data:
                    if entry["color"] == color_name:
                        return np.array(entry["mean"]), np.array(entry["covariance"])
            else:  # Single JSON object case
                if data["color"] == color_name:
                    return np.array(data["mean"]), np.array(data["covariance"])
        except json.JSONDecodeError as e:
            print("Error loading JSON:", e)
            return None, None

    raise ValueError(f"Color '{color_name}' not found in {filepath}")



def test_retreive():
    og_mu, og_sig = multivariate_gaussian("color_data/red_data.csv")
    print("OG MU: ", og_mu)
    print("OG SIGMA: ", og_sig, '\n')

    av, sig = retreive_mult_gaussian("red")
    print("GOT MU: ", av)
    print("GOT SIGMA: ", sig, '\n')


    assert (og_mu == av).all()
    assert (og_sig == sig).all()

    

if __name__ == "__main__":
    #save_mult_gaussian_for_all_colors()

    test_retreive()