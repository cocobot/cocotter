# Simulateur cocotter

Simulateur 3D des robots (galipeur, pami, adversaire) basé sur Bevy.

## Prérequis

- Rust toolchain stable (`rustup`)
- Node.js + npx (pour l'optimisation des modèles 3D, optionnel)
- Clés API Onshape si vous voulez récupérer les modèles 3D depuis le CAD

## Préparation des assets

### Récupérer le modèle 3D galipeur depuis Onshape

```bash
export ONSHAPE_ACCESS_KEY="votre_clé"
export ONSHAPE_SECRET_KEY="votre_secret"
./tool/onshape_fetch/scripts/fetch-galipeur.sh
```

Cela produit les fichiers `.glb` dans `sim/assets/robots/galipeur/` (base + joints articulés).

> Si les modèles 3D ne sont pas présents, le simulateur affiche un cuboid placeholder avec un warning dans les logs.

## Lancement

### Simulateur seul

```bash
cargo run -p sim
```

### Avec un robot au démarrage

```bash
cargo run -p sim -- --spawn galipeur:left
cargo run -p sim -- --spawn galipeur:left --spawn pami:right
cargo run -p sim -- --spawn galipeur:left --spawn adversary
```

### Lancer un robot manuellement (dans un autre terminal)

```bash
cargo rgalipeur    # alias pour: cargo run -p galipeur
cargo rpami        # alias pour: cargo run -p pami
```

## Options CLI

| Option | Description |
|--------|-------------|
| `--spawn KIND[:SIDE]` | Spawn un robot au démarrage (`galipeur:left`, `pami:right`, `adversary`) |
| `--field PATH` | Fichier TOML de config terrain (défaut: `sim/cfg/table.toml`) |
| `--headless` | Mode sans fenêtre (pour tests d'intégration) |
| `--time-factor N` | Vitesse de simulation (1.0 = temps réel, 5.0 = accéléré) |
| `--auto-start` | Lance automatiquement la séquence starter une fois les robots connectés |
| `--socket PATH` | Chemin du socket Unix (défaut: `/tmp/cocotter_sim.sock`) |

## Raccourcis clavier

Les raccourcis sont affichés dans le simulateur via le HUD (dock en bas).
Principaux :

- **N puis G/P/A** : spawn galipeur / pami / adversaire
- **T** : insérer/retirer le starter (lance le match)
- **V** : afficher/masquer les debug volumes
- **R** : reset (recharge la config et relance)
- **ZQSD** : déplacer l'adversaire manuellement

## Configuration

Le fichier `sim/cfg/table.toml` contient :

- Dimensions du terrain
- Obstacles
- Poses de départ des robots
- Cinématique holonome (galipeur)
- Modèle 3D et joints articulés
- Configuration des humains (spectateurs aléatoires)
