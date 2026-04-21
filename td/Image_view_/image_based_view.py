import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def show_image(image):
    cv.imshow("Image", image)
    cv.waitKey(0)
    cv.destroyAllWindows()

def plot_error_fct(error):
    fig = plt.figure(figsize=(10, 8))
    ax = plt.axes(projection='3d')
    
    # Create meshgrid for X and Y coordinates
    X = np.arange(-Tx_var, Tx_var + 1)
    Y = np.arange(-Ty_var, Ty_var + 1)
    X, Y = np.meshgrid(X, Y)
    
    # Plot surface with colormap
    surf = ax.plot_surface(X, Y, error.T, cmap='viridis', edgecolor='none', alpha=0.9)
    
    # Add labels and title
    ax.set_xlabel('Translation X (pixels)')
    ax.set_ylabel('Translation Y (pixels)')
    ax.set_zlabel('SSD Error')
    ax.set_title('Sum of Squared Differences Error Surface')
    
    # Add colorbar
    fig.colorbar(surf, shrink=0.5, aspect=10)
    
    plt.show()

################################################################################################################################

image = cv.imread("image.png", cv.IMREAD_GRAYSCALE)
if image is None:
    raise ValueError("Image not found")
image_width = image.shape[1]
image_height = image.shape[0]
#show_image(image)

window_center = (107, 128)
window_size = (80, 100)
I_star = image[window_center[1] - window_size[1] // 2:window_center[1] + window_size[1] // 2, window_center[0] - window_size[0] // 2:window_center[0] + window_size[0] // 2]
#show_image(I_star)

I_star_width = I_star.shape[1]
I_star_height = I_star.shape[0]

Tx_var = 40
Ty_var = 40

errors = np.zeros((2 * Tx_var + 1, 2 * Ty_var + 1))

for i in range(-Tx_var, Tx_var + 1):
    for j in range(-Ty_var, Ty_var + 1):
        image_ = image[window_center[1] - window_size[1] // 2 + i : window_center[1] + window_size[1] // 2 + i,
                       window_center[0] - window_size[0] // 2 + j : window_center[0] + window_size[0] // 2 + j]
        errors[i + Tx_var, j + Ty_var] = np.sum((image_ - I_star)**2)


#plot_error_fct(errors)

################################################################################################################################
# Image based view implementation

# 1. Calcul des gradients de l'image de référence I_star 
I_star_grad_u = cv.Sobel(I_star, cv.CV_64F, 1, 0, ksize=3)
I_star_grad_v = cv.Sobel(I_star, cv.CV_64F, 0, 1, ksize=3)

# 2. Paramètres intrinsèques (le prof dis par approximation on a )
u0 = image_width / 2
v0 = image_height / 2
f = image_width

lambda_gain = 1.5
Z = 1.0

# 3. Préparation des coordonnées (u, v) pour chaque pixel de la fenêtre
wy, wx = window_size
y_indices, x_indices = np.mgrid[0:wy, 0:wx]
u = (window_center[0] - wx // 2) + x_indices
v = (window_center[1] - wy // 2) + y_indices

# Coordonnées normalisées (x, y)
x_norm = (u - u0) / f
y_norm = (v - v0) / f

# 4. Construction de la Matrice d'Interaction L_I (taille N_pixels x 6)
# L_I = - [ dI/du  dI/dv ] * L_xy
# L_xy est la matrice d'interaction géométrique pour un point (2x6)

# On vectorise les calculs pour aller plus vite (N pixels)
N = wx * wy
I_du = I_star_grad_u.flatten()
I_dv = I_star_grad_v.flatten()
xn = x_norm.flatten()
yn = y_norm.flatten()

# Matrice d'interaction pour chaque pixel (L_xy)
# L_s = [-1/Z, 0, x/Z, xy, -(1+x^2), y]
#       [0, -1/Z, y/Z, 1+y^2, -xy, -x]

L_I = np.zeros((N, 6))

# Remplissage colonnes par colonnes selon la formule L_I = - (Ix * L_x + Iy * L_y)
# Col 1: -1/Z
L_I[:, 0] = - (I_du * (-1./Z) + I_dv * 0)
# Col 2: 0, -1/Z
L_I[:, 1] = - (I_du * 0 + I_dv * (-1./Z))
# Col 3: x/Z, y/Z
L_I[:, 2] = - (I_du * (xn/Z) + I_dv * (yn/Z))
# Col 4: xy, 1+y^2
L_I[:, 3] = - (I_du * (xn*yn) + I_dv * (1 + yn**2))
# Col 5: -(1+x^2), -xy
L_I[:, 4] = - (I_du * (-(1 + xn**2)) + I_dv * (-xn*yn))
# Col 6: y, -x
L_I[:, 5] = - (I_du * yn + I_dv * (-xn))

# Pseudo-inverse de la matrice d'interaction 
L_I_pinv = np.linalg.pinv(L_I)

# Stockage des vitesses calculées pour affichage
velocities = np.zeros((2 * Tx_var + 1, 2 * Ty_var + 1, 6))

while errors[Tx_var, Ty_var] > 1:


    for i in range(-Tx_var, Tx_var + 1):
        for j in range(-Ty_var, Ty_var + 1):
            # Image courante (décalée)
            image_curr = image[window_center[1] - window_size[1] // 2 + i : window_center[1] + window_size[1] // 2 + i,
                            window_center[0] - window_size[0] // 2 + j : window_center[0] + window_size[0] // 2 + j]
            
            # Vecteur erreur (différence de luminance pixel à pixel)
            error_vec = (image_curr.astype(float) - I_star.astype(float)).flatten()
            
            # Calcul de la vitesse de la caméra : v = - lambda * L_pinv * erreur
            v_camera = - lambda_gain * (L_I_pinv @ error_vec)
            
            # On sauvegarde la vitesse (par exemple on veut voir si ça converge vers 0 quand i,j = 0,0)
            velocities[i + Tx_var, j + Ty_var] = v_camera
            
    print(velocities.shape)



# --- 3. BOUCLE DE SIMULATION DYNAMIQUE ---

# Position initiale (DÉCALÉE pour voir l'évolution)
# On décale de 15 pixels en X et 10 en Y
current_pos = window_center + np.array([15.0, 10.0]) 

lambda_gain = 2.0  # Gain de contrôle (vitesse de convergence)
iter_max = 100
error_history = []

plt.ion() # Mode interactif pour matplotlib
fig, ax = plt.subplots(1, 3, figsize=(15, 5))

print(f"Départ: {current_pos}, Cible: {window_center}")

for k in range(iter_max):
    # A. Acquisition de l'image courante    
    # cv.getRectSubPix gère l'interpolation si current_pos est un float
    I_current = cv.getRectSubPix(image , (window_size[0], window_size[1]), tuple(current_pos))
    
    # B. Calcul de l'erreur photométrique
    error_img = I_current.astype(float) - I_star.astype(float)
    error_vec = error_img.flatten()
    
    # Norme de l'erreur (SSD moyen)
    ssd = np.mean(error_vec**2)
    error_history.append(ssd)
    
    # C. Loi de Commande : v = -lambda * L_pinv * e
    v_camera = -lambda_gain * (L_I_pinv @ error_vec)
    
    # D. Mise à jour de la position (Simulation)
    # On ne garde que v_x et v_y (indices 0 et 1) car on simule un mouvement plan
    # Note : Si la caméra bouge à droite (+v_x), la fenêtre de vue bouge aussi (+x)
    dx = v_camera[0]
    dy = v_camera[1]
    
    current_pos[0] += dx
    current_pos[1] += dy
    
    # --- VISUALISATION ---
    if k % 2 == 0: # Mise à jour tous les 2 frames pour fluidité
        ax[0].clear()
        ax[0].imshow(I_current, cmap='gray', vmin=0, vmax=255)
        ax[0].set_title(f"Vue Courante (Iter {k})")
        
        ax[1].clear()
        # Affichage différence (gris moyen = 0 erreur)
        ax[1].imshow(error_img, cmap='seismic', vmin=-100, vmax=100)
        ax[1].set_title("Différence (I - I*)")
        
        ax[2].clear()
        ax[2].plot(error_history, 'r-')
        ax[2].set_title(f"SSD Error: {ssd:.2f}")
        ax[2].set_xlabel("Itérations")
        ax[2].grid(True)
        
        plt.pause(0.05)
    
    # Critère d'arrêt
    if ssd < 1.0:
        print(f"Convergence atteinte à l'itération {k} !")
        break

plt.ioff()
plt.show()