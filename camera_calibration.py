#!/usr/bin/env python3
import os
import cv2
import numpy as np

# ==========================================
# CONFIGURAZIONE PARAMETRI (Inserisci le tue misure)
# ==========================================
CHESSBOARD_SIZE = (7, 5)   # Numero di angoli interni (Colonne, Righe)
SQUARE_SIZE_MM = 80.0     # Dimensione reale del lato del quadrato in millimetri
IMAGES_PATH = os.path.expanduser('~/ros2_ws_AA/calibration_data/')
# ==========================================

def run_offline_calibration():
    # Preparazione delle coordinate 3D teoriche (0,0,0), (25,0,0), (50,0,0)...
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    objpoints = []  # Punti nello spazio 3D reale
    imgpoints = []  # Punti sul piano 2D dell'immagine

    # Recupera tutte le immagini salvate nella cartella
    if not os.path.exists(IMAGES_PATH):
        print(f"Errore: La cartella {IMAGES_PATH} non esiste!")
        return

    images = [os.path.join(IMAGES_PATH, f) for f in os.listdir(IMAGES_PATH) if f.endswith('.jpg')]
    images.sort()

    if len(images) == 0:
        print(f"Nessuna immagine .jpg trovata in {IMAGES_PATH}")
        return
    
    print(f"Trovate {len(images)} immagini. Inizio elaborazione di debug...")

    gray = None
    images_used = 0

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Impossibile leggere l'immagine: {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Cerca i punti interni
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if ret:
            objpoints.append(objp)
            
            # Subpixel refinement per aumentare la precisione millimetrica
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            images_used += 1
            print(f"[OK] Punti trovati in: {os.path.basename(fname)}")
            
            # Mostra l'immagine di debug per vedere se l'algoritmo ha preso i punti giusti
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
            cv2.imshow("Debug Calibrazione (Premi un tasto per andare avanti)", cv2.resize(img, (800, 600)))
            cv2.waitKey(200) # Mostra per 200ms prima di passare alla successiva
        else:
            print(f"[FALLITO] Scacchiera NON trovata in: {os.path.basename(fname)} (Verrà scartata)")

    cv2.destroyAllWindows()

    if images_used < 10:
        print(f"\nAttenzione: Sono state usate solo {images_used} immagini. Consigliate almeno 10-15 foto valide.")
    
    if len(objpoints) > 0:
        print("\nCalcolo della matrice di calibrazione in corso...")
        # Calibrazione
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
             objpoints, imgpoints, gray.shape[::-1],
             None, None,
             flags=cv2.CALIB_RATIONAL_MODEL
        )
        
        print("\n=============================================")
        print("          RISULTATI DELLA CALIBRAZIONE       ")
        print("=============================================")
        print(f"Errore di riproiezione totale: {ret:.4f} pixel (Ottimale se < 0.5)")
        print("\nMatrice Intrinseca della Camera (K):")
        print(mtx)
        print("\nCoefficienti di Distorsione (D):")
        print(dist.ravel())
        print("=============================================\n")
        
        # Salva i dati in un file di testo leggibile
        output_file = os.path.join(IMAGES_PATH, "calibration_results.txt")
        with open(output_file, "w") as f:
            f.write(f"--- SIYI A8 Calibration Results ---\n")
            f.write(f"Reprojection Error: {ret}\n\n")
            f.write(f"Camera Matrix (K):\n{mtx}\n\n")
            f.write(f"Distortion Coefficients (D):\n{dist.ravel()}\n")
            
        print(f"I risultati sono stati salvati con successo in:\n{output_file}")
    else:
        print("\nErrore critico: Nessuna delle foto elaborate conteneva una scacchiera valida.")

if __name__ == '__main__':
    run_offline_calibration()
