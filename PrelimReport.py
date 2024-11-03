import numpy as np
from scipy.io import savemat

# Connection matrix (example based on the document)
C = np.array([
    [1, 1, 0, 0, 0, 0, 0],
    [1, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 1, 0, 0, 0],
    [0, 0, 0, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 1, 1]
])

# Support force matrices (example for one pin and one roller)
Sx = np.array([1, 0, 0, 0, 0])
Sy = np.array([0, 1, 0, 0, 1])

# Joint coordinates (example values)
X = np.array([0, 7, 14, 21, 28])  # X-coordinates of each joint
Y = np.array([0, 0, 0, 0, 0])     # Y-coordinates (assuming all joints are on the same line)

# Load vector (assuming a load at joint 2 in the y-direction)
L = np.zeros(2 * len(X))
L[6] = -2  # -2 lbs downward load on joint 2 in y-axis (negative means downward)

def construct_equilibrium_matrix(C, X, Y, Sx, Sy):
    J = len(X)  # Number of joints
    M = C.shape[1]  # Number of members
    A = np.zeros((2 * J, M + 3))  # The matrix A will have 2J rows and M+3 columns
    
    # Construct A based on equilibrium equations
    for j in range(J):
        for m in range(M):
            if C[j, m] == 1:
                x_start, y_start = X[j], Y[j]
                x_end, y_end = X[j + 1], Y[j + 1] if j + 1 < J else (0, 0)
                length = np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
                
                # Calculate x and y direction cosines
                cos_x = (x_end - x_start) / length if length != 0 else 0
                cos_y = (y_end - y_start) / length if length != 0 else 0
                
                # Populate A matrix for x and y equilibrium
                A[j, m] = cos_x
                A[J + j, m] = cos_y

        # Add reaction forces to A
        A[j, -3] = Sx[j]
        A[J + j, -2] = Sy[j]

    return A

# Construct A matrix
A = construct_equilibrium_matrix(C, X, Y, Sx, Sy)

def solve_truss(A, L):
    try:
        # Solve for forces (T)
        T = np.linalg.solve(A, L)
        return T
    except np.linalg.LinAlgError:
        print("Matrix is singular, cannot solve.")
        return None

# Solve for forces
T = solve_truss(A, L)
if T is not None:
    print("\nResults for Truss Design 1")

    # Calculate truss cost and load-to-cost ratio
    def calculate_truss_cost(C, X, Y):
        # Cost constants
        cost_per_joint = 10
        cost_per_length = 1
        J = len(X)
        M = C.shape[1]
        
        total_length = 0
        for m in range(M):
            joint_indices = np.where(C[:, m] == 1)[0]
            if len(joint_indices) == 2:
                x1, y1 = X[joint_indices[0]], Y[joint_indices[0]]
                x2, y2 = X[joint_indices[1]], Y[joint_indices[1]]
                member_length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                total_length += member_length

        total_cost = J * cost_per_joint + total_length * cost_per_length
        return total_cost

    # Calculate cost and load-to-cost ratio
    truss_cost = calculate_truss_cost(C, X, Y)
    load_cost_ratio = abs(L[6]) / truss_cost  # Assuming live load applied at joint 2 in y-direction

    print(f"\nTruss cost: ${truss_cost:.2f}")
    print(f"Load-to-cost ratio: {load_cost_ratio:.4f}")
    
    # Display member forces and reactions
    print("\nMember Forces (in lbs):")
    for i in range(len(T) - 3):  # Skip the last three as they are support reactions
        force_type = "Tension" if T[i] > 0 else "Compression"
        print(f"  Member {i+1}: {abs(T[i]):.2f} ({force_type})")

    print("\nReaction Forces:")
    print(f"  Sx at Joint A: {T[-3]:.2f} lbs")
    print(f"  Sy at Joint A: {T[-2]:.2f} lbs")
    print(f"  Sy at Joint B: {T[-1]:.2f} lbs")

# Save input to .mat file for submission
def save_truss_input(filename, C, Sx, Sy, X, Y, L):
    data = {
        'C': C,
        'Sx': Sx,
        'Sy': Sy,
        'X': X,
        'Y': Y,
        'L': L
    }
    savemat(filename, data)
    print(f"Input data saved to {filename}")

save_truss_input("TrussDesign1.mat", C, Sx, Sy, X, Y, L)
