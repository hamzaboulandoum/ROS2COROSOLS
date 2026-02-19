import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec

def plot_pid_data(filename='theta_pid_log.csv'):
    """
    Plot les donn�es PID pour analyser les performances du contr�leur
    """
    try:
        # Lire les donn�es CSV
        df = pd.read_csv(filename)
        
        # Convertir le timestamp en temps relatif (secondes)
        df['time'] = df['timestamp'] - df['timestamp'].iloc[0]
        
        # Cr�er la figure avec plusieurs sous-graphiques
        fig = plt.figure(figsize=(15, 12))
        gs = GridSpec(4, 2, figure=fig)
        
        # 1. Erreur d'orientation (theta_error)
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(df['time'], df['theta_error'], 'b-', linewidth=1.5, label='Erreur d\'orientation (�)')
        ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='R�f�rence (0�)')
        ax1.set_ylabel('Erreur (�)')
        ax1.set_title('Erreur d\'orientation du robot')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Sortie du contr�leur PID (theta_output)
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.plot(df['time'], df['theta_output'], 'g-', linewidth=1.5, label='Sortie PID')
        ax2.set_ylabel('Sortie PID')
        ax2.set_title('Sortie du contr�leur PID')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Commande de vitesse de rotation (vr_command)
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.plot(df['time'], df['vr_command'], 'r-', linewidth=1.5, label='Commande vr')
        ax3.set_ylabel('Vitesse de rotation (rad/s)')
        ax3.set_title('Commande de vitesse de rotation')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. Pas de temps (dt)
        ax4 = fig.add_subplot(gs[2, 0])
        ax4.plot(df['time'], df['dt'], 'm-', linewidth=1.5, label='dt')
        ax4.set_ylabel('dt (s)')
        ax4.set_title('Pas de temps')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. Fr�quence d'�chantillonnage
        ax5 = fig.add_subplot(gs[2, 1])
        frequency = 1.0 / df['dt']
        ax5.plot(df['time'], frequency, 'c-', linewidth=1.5, label='Fr�quence')
        ax5.set_ylabel('Fr�quence (Hz)')
        ax5.set_title('Fr�quence d\'�chantillonnage')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. Analyse des gains PID
        ax6 = fig.add_subplot(gs[3, :])
        ax6.plot(df['time'], df['kp'], 'b-', linewidth=1.5, label=f'Kp = {df["kp"].iloc[0]:.4f}')
        ax6.plot(df['time'], df['ki'], 'g-', linewidth=1.5, label=f'Ki = {df["ki"].iloc[0]:.4f}')
        ax6.plot(df['time'], df['kd'], 'r-', linewidth=1.5, label=f'Kd = {df["kd"].iloc[0]:.4f}')
        ax6.set_xlabel('Temps (s)')
        ax6.set_ylabel('Gains PID')
        ax6.set_title('Gains PID utilis�s')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Afficher des statistiques
        print("\n=== STATISTIQUES PID ===")
        print(f"Dur�e totale: {df['time'].iloc[-1]:.2f} secondes")
        print(f"Nombre d'�chantillons: {len(df)}")
        print(f"Fr�quence moyenne: {frequency.mean():.1f} Hz")
        print(f"Erreur maximale: {df['theta_error'].max():.2f}�")
        print(f"Erreur minimale: {df['theta_error'].min():.2f}�")
        print(f"Erreur RMS: {np.sqrt((df['theta_error']**2).mean()):.2f}�")
        print(f"Sortie PID max: {df['theta_output'].max():.4f}")
        print(f"Sortie PID min: {df['theta_output'].min():.4f}")
        print(f"Commande vr max: {df['vr_command'].max():.4f}")
        print(f"Commande vr min: {df['vr_command'].min():.4f}")
        
        # V�rifier si la commande a atteint les limites
        vr_limited = (df['vr_command'] == 0.2) | (df['vr_command'] == -0.2)
        if vr_limited.any():
            print(f"??  ATTENTION: La commande a atteint les limites {vr_limited.sum()} fois")
            print("   Consid�rez augmenter les gains ou les limites")
        
    except FileNotFoundError:
        print(f"? Fichier {filename} non trouv�!")
        print("Assurez-vous que la simulation a �t� ex�cut�e au moins une fois.")
    except Exception as e:
        print(f"? Erreur lors de la lecture du fichier: {e}")

def plot_pid_comparison(filename1='theta_pid_log.csv', filename2=None):
    """
    Compare les donn�es PID entre deux fichiers (utile pour comparer diff�rents r�glages)
    """
    try:
        df1 = pd.read_csv(filename1)
        df1['time'] = df1['timestamp'] - df1['timestamp'].iloc[0]
        
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # Plot erreur d'orientation
        axes[0].plot(df1['time'], df1['theta_error'], 'b-', linewidth=1.5, label=f'Test 1 (Kp={df1["kp"].iloc[0]:.4f})')
        if filename2:
            df2 = pd.read_csv(filename2)
            df2['time'] = df2['timestamp'] - df2['timestamp'].iloc[0]
            axes[0].plot(df2['time'], df2['theta_error'], 'r-', linewidth=1.5, label=f'Test 2 (Kp={df2["kp"].iloc[0]:.4f})')
        
        axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[0].set_ylabel('Erreur (�)')
        axes[0].set_title('Comparaison des erreurs d\'orientation')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        # Plot commande de vitesse
        axes[1].plot(df1['time'], df1['vr_command'], 'b-', linewidth=1.5, label='Test 1')
        if filename2:
            axes[1].plot(df2['time'], df2['vr_command'], 'r-', linewidth=1.5, label='Test 2')
        
        axes[1].set_xlabel('Temps (s)')
        axes[1].set_ylabel('Vitesse de rotation (rad/s)')
        axes[1].set_title('Comparaison des commandes de vitesse')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"? Erreur lors de la comparaison: {e}")

if __name__ == "__main__":
    print("=== ANALYSEUR DE DONN�ES PID ===")
    print("1. Plotter les donn�es PID")
    print("2. Comparer deux fichiers PID")
    
    choice = input("Choisissez une option (1 ou 2): ").strip()
    
    if choice == "1":
        plot_pid_data()
    elif choice == "2":
        file1 = input("Nom du premier fichier (d�faut: theta_pid_log.csv): ").strip() or "theta_pid_log.csv"
        file2 = input("Nom du deuxi�me fichier (optionnel): ").strip() or None
        plot_pid_comparison(file1, file2)
    else:
        print("Option invalide. Utilisation de l'option 1 par d�faut.")
        plot_pid_data()