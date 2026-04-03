# Protocole CAN Cocotter

## Format des IDs CAN

Format 11-bit Standard:

```
ID = 0x[DOMAIN][CMD][TARGET]
      │        │     │
      │        │     └── Cible (0-F) - interprétation dépend du domaine
      │        └──────── Commande (0-F) dans le domaine
      └───────────────── Domaine (0-7)
```

Bits: `[DOMAIN:3bits][CMD:4bits][TARGET:4bits]` = 11 bits total.

## Domaines

| Domain | Nom | Description |
|--------|-----|-------------|
| 0x0 | SYSTEM | Ping, erreurs, infos système |
| 0x1 | ARM | Commandes bras (servo/pompe/valve) |
| 0x2 | GROUND | Capteurs de sol |
| 0x3 | LOG | Logs picotter → sabotter |
| 0x4 | OTA | Mise à jour firmware |
| 0x5-0x7 | Réservé | Extensions futures |

**Direction:**
- P→S : Sabotter (PC/master) envoie à Picotter (board/slave)
- S→P : Picotter envoie à Sabotter

---

## Domaine 0x0 - SYSTEM

Messages système généraux. Target = 0 sauf pour les réponses (target = 1).

### Commandes

| Cmd | Nom | Valeur |
|-----|-----|--------|
| 0x0 | Ping | `SystemCmd::Ping` |
| 0x1 | BoardInfo | `SystemCmd::BoardInfo` |
| 0x2 | SetServoId | `SystemCmd::SetServoId` |
| 0xE | Error | `SystemCmd::Error` |
| 0xF | Reboot | `SystemCmd::Reboot` |

### 0x000 - PING

Test de communication.

```
ID: 0x000
Longueur: 1 octet
Direction: Both

Data[0]: Valeur

Réponse:
Data[0]: Valeur + 1
```

### 0x010 - BOARD_INFO (requête)

Demande d'informations sur la board.

```
ID: 0x010 (target=0)
Longueur: 0 octet
Direction: P→S
```

### 0x011 - BOARD_INFO (réponse)

Informations sur la board.

```
ID: 0x011 (target=1)
Longueur: 8 octets
Direction: S→P

Data[0-3]: Version firmware (u32, LE) - format: MAJOR.MINOR.PATCH.BUILD
Data[4-7]: Uptime en ms (u32, LE)
```

### 0x020 - SET_SERVO_ID

Configure l'ID d'un servo par broadcast sur un bus spécifique.
⚠️ **Important:** Un seul servo doit être connecté sur le bus lors de l'exécution.

```
ID: 0x020 (target=0)
Longueur: 2 octets
Direction: P→S

Data[0]: Bus cible (ServoBus)
         - 0 = Module 0 (4 servos bras)
         - 1 = Module 1 (4 servos bras)
         - 2 = Module 2 (4 servos bras)
         - 3 = Translation (3 servos)
Data[1]: Nouvel ID servo (1-253)
```

### 0x021 - SET_SERVO_ID_RESULT

Résultat de la commande SET_SERVO_ID.

```
ID: 0x021 (target=1)
Longueur: 3 octets
Direction: S→P

Data[0]: Bus utilisé (0-3)
Data[1]: ID configuré
Data[2]: Résultat (0=échec, 1=succès)
```

### 0x0E0 - ERROR

Rapport d'erreur asynchrone.

```
ID: 0x0E0
Longueur: 2 octets
Direction: S→P

Data[0]: Type d'erreur (ErrorType)
         - 0x01 = Servo communication
         - 0x02 = I2C error
         - 0x03 = Invalid command
         - 0x04 = CAN overflow
         - 0x10 = OTA error
Data[1]: Code détaillé
```

### 0x0F0 - REBOOT

Commande de redémarrage.

```
ID: 0x0F0
Longueur: 1 octet
Direction: P→S

Data[0]: Mode de reboot (RebootMode)
         - 0 = Normal (application)
         - 1 = Bootloader (pour flash manuel)
```

---

## Domaine 0x1 - ARM

Commandes pour les bras (servo + pompe + électrovanne).

**Encodage du target (4 bits) — index plat :**

Le système comporte 3 modules de 4 bras chacun (12 bras total). Le target est encodé
comme un index plat :

```
target = module × 4 + arm    (valeurs 0-11)
target = 0xF                  (broadcast tous modules/bras)
```

| Module | Arm | Target |
|--------|-----|--------|
| 0 | 0 | 0x0 |
| 0 | 1 | 0x1 |
| 0 | 2 | 0x2 |
| 0 | 3 | 0x3 |
| 1 | 0 | 0x4 |
| 1 | 1 | 0x5 |
| 1 | 2 | 0x6 |
| 1 | 3 | 0x7 |
| 2 | 0 | 0x8 |
| 2 | 1 | 0x9 |
| 2 | 2 | 0xA |
| 2 | 3 | 0xB |
| Tous | Tous | 0xF |

Décodage : `module = target / 4`, `arm = target % 4`.

### Commandes

| Cmd | Nom | Valeur |
|-----|-----|--------|
| 0x0 | SetArm | `ArmCmd::SetArm` |
| 0x1 | Status | `ArmCmd::Status` |
| 0x2 | RequestStatus | `ArmCmd::RequestStatus` |
| 0x3 | SetTorque | `ArmCmd::SetTorque` |
| 0x4 | SetPump | `ArmCmd::SetPump` |
| 0x5 | SetValve | `ArmCmd::SetValve` |
| 0x6 | SetTranslation | `ArmCmd::SetTranslation` |
| 0x7 | TranslationStatus | `ArmCmd::TranslationStatus` |
| 0x8 | SetColorConfig | `ArmCmd::SetColorConfig` |
| 0x9 | SetColorSensorConfig | `ArmCmd::SetColorSensorConfig` |
| 0xA | ColorSensorRaw | `ArmCmd::ColorSensorRaw` |
| 0xB | SetColorLedPwm | `ArmCmd::SetColorLedPwm` |
| 0xC | RequestTranslationStatus | `ArmCmd::RequestTranslationStatus` |
| 0xD | SetStage2 | `ArmCmd::SetStage2` |
| 0xE | SetStage2Torque | `ArmCmd::SetStage2Torque` |
| 0xF | Stage2Status | `ArmCmd::Stage2Status` |

### 0x10T - SET_ARM

Commande complète d'un bras.

```
ID: 0x10[target]
Longueur: 6 octets
Direction: P→S

Data[0-1]: Position servo (0-1023), little-endian
Data[2-3]: Temps de mouvement (ms), little-endian (0 = vitesse max)
Data[4]:   Pompe (0=OFF, 1=ON)
Data[5]:   Électrovanne (0=OFF, 1=ON)
```

**Exemples:**
| Action | ID | Data (hex) |
|--------|-----|------------|
| Module 0, Bras 2 (target=2): pos=500, 200ms, pompe ON, EV OFF | `0x102` | `F4 01 C8 00 01 00` |
| Module 1, Bras 0 (target=4): pos=512, 100ms, tout OFF | `0x104` | `00 02 64 00 00 00` |
| Broadcast tous bras (target=F) | `0x10F` | `...` |

### 0x11T - ARM_STATUS

Status d'un bras. Envoyé périodiquement ou en réponse à REQUEST_STATUS.

```
ID: 0x11[target]
Longueur: 8 octets
Direction: S→P

Data[0-1]: Position servo actuelle (0-1023), little-endian
Data[2]:   Identifiant couleur (u8, 0=aucune couleur, 255=config incomplète, 1-254=color_id)
Data[3]:   État pompe (0=OFF, 1=ON)
Data[4]:   État électrovanne (0=OFF, 1=ON)
Data[5]:   Code erreur servo (0=OK, voir codes ci-dessous)
Data[6]:   Flags (ArmFlags)
           - bit 0: Torque enabled
           - bit 1: En mouvement
           - bit 2: Position atteinte
Data[7]:   Courant pompe (ADC 12-bit >> 4)
```

**Codes erreur servo:**
| Code | Signification |
|------|---------------|
| 0x00 | OK |
| 0x01 | Timeout communication |
| 0x10 | Voltage error |
| 0x20 | Overload |
| 0x40 | Overheating |

### 0x12T - REQUEST_STATUS

Force l'envoi immédiat du status.

```
ID: 0x12[target]
Longueur: 0 octet
Direction: P→S

Réponse: Message 0x11T correspondant
```

### 0x13T - SET_TORQUE

Active ou désactive le couple du servo.

```
ID: 0x13[target]
Longueur: 1 octet
Direction: P→S

Data[0]: 0=Disable, 1=Enable
```

### 0x14T - SET_PUMP

Commande pompe seule.

```
ID: 0x14[target]
Longueur: 1 octet
Direction: P→S

Data[0]: 0=OFF, 1=ON
```

### 0x15T - SET_VALVE

Commande électrovanne seule.

```
ID: 0x15[target]
Longueur: 1 octet
Direction: P→S

Data[0]: 0=OFF, 1=ON
```

### 0x16M - SET_TRANSLATION

Commande le servo de translation d'un module.
Les servos de translation sont sur un bus partagé (USART1), un par module.

Le target est directement le numéro de module (0-2).

```
ID: 0x16[module]
Longueur: 4 octets
Direction: P→S

Data[0-1]: Position servo (0-1023), little-endian
Data[2-3]: Temps de mouvement (ms), little-endian (0 = vitesse max)
```

**Servo IDs de translation (configurables):**
| Module | Servo ID par défaut |
|--------|---------------------|
| 0 | 10 |
| 1 | 11 |
| 2 | 12 |

### 0x17M - TRANSLATION_STATUS

Status du servo de translation d'un module.

Le target est directement le numéro de module (0-2).

```
ID: 0x17[module]
Longueur: 3 octets
Direction: S→P

Data[0-1]: Position actuelle (0-1023), little-endian
Data[2]:   Code erreur (0=OK)
```

### 0x1CM - REQUEST_TRANSLATION_STATUS

Force l'envoi immédiat du status de translation.

```
ID: 0x1C[module]
Longueur: 0 octet
Direction: P→S

Réponse: Message 0x17M correspondant
```

### 0x1DT - SET_STAGE2

Commande le servo du 2ème étage.
Chaque module possède 2 servos de 2ème étage sur le même bus UART que les bras.

Le target utilise un encodage plat : `module * 2 + servo` (0-5), 0xF = broadcast.

```
ID: 0x1D[target]
Longueur: 4 octets
Direction: P→S

Data[0-1]: Position servo (0-1023), little-endian
Data[2-3]: Temps de mouvement (ms), little-endian (0 = vitesse max)
```

**Servo IDs de 2ème étage (configurables, par défaut):**
| Module | Servo 0 | Servo 1 |
|--------|---------|---------|
| 0 | 20 | 21 |
| 1 | 20 | 21 |
| 2 | 20 | 21 |

### 0x1ET - SET_STAGE2_TORQUE

Active ou désactive le couple d'un servo du 2ème étage.

```
ID: 0x1E[target]
Longueur: 1 octet
Direction: P→S

Data[0]: 0=OFF, 1=ON
```

### 0x1FT - STAGE2_STATUS

Status d'un servo du 2ème étage. Envoyé périodiquement ou en réponse à une requête.

Requête (P→S) : 0 octet de données.
Réponse (S→P) : 4 octets.

```
ID: 0x1F[target]
Longueur: 0 octets (requête) ou 4 octets (status)
Direction: P→S (requête) / S→P (status)

Data[0-1]: Position servo actuelle (0-1023), little-endian
Data[2]:   Code erreur servo (0=OK)
Data[3]:   Flags (ArmFlags)
           - bit 0: Torque enabled
           - bit 1: En mouvement
           - bit 2: Position atteinte
```

### 0x18T - SET_COLOR_CONFIG

Configure une plage de valeurs pour un canal d'un color_id donné.
4 messages sont nécessaires pour configurer une couleur complète (un par canal).

Chaque bras possède une table de décodage (max 8 couleurs). Pour qu'une couleur soit
détectée, les 4 canaux (C, R, G, B) doivent être dans leur plage respective.

```
ID: 0x18[target]
Longueur: 6 octets
Direction: P→S

Data[0]: Color ID (1-254 = configurer, 0 = effacer toute la table)
Data[1]: Canal (0=Clear, 1=Red, 2=Green, 3=Blue)
Data[2-3]: Valeur minimum (u16, LE)
Data[4-5]: Valeur maximum (u16, LE)
```

**Valeurs de retour couleur (dans ARM_STATUS):**
| Valeur | Signification |
|--------|---------------|
| 255 | Config incomplète (capteur ou table non configuré) |
| 0 | Aucune couleur ne matche |
| 1-254 | Color ID détecté |

### 0x19T - SET_COLOR_SENSOR_CONFIG

Configure les paramètres du capteur TCS3472 (integration time + gain).
Un capteur par bras, accessible via mux I2C TCA9548A.

```
ID: 0x19[target]
Longueur: 2 octets
Direction: P→S

Data[0]: Integration time (registre ATIME brut)
         - 0xFF = 2.4ms
         - 0xD5 = ~101ms
         - 0xC0 = ~154ms
         - 0x00 = ~614ms
         Formule: temps_ms = (256 - ATIME) × 2.4
Data[1]: Gain
         - 0 = 1x
         - 1 = 4x
         - 2 = 16x
         - 3 = 60x
```

### 0x1AT - COLOR_SENSOR_RAW

Lecture des valeurs brutes du capteur TCS3472 (debug).

**Requête (P→S):**
```
ID: 0x1A[target]
Longueur: 0 octet
```

**Réponse (S→P):**
```
ID: 0x1A[target]
Longueur: 8 octets

Data[0-1]: Clear (u16, LE)
Data[2-3]: Red (u16, LE)
Data[4-5]: Green (u16, LE)
Data[6-7]: Blue (u16, LE)
```

### 0x1B0 - SET_COLOR_LED_PWM

Configure le duty cycle de la PWM pour les LEDs des capteurs de couleur TCS3472.
Une seule PWM (TIM8_CH3, PC8) contrôle toutes les LEDs.

```
ID: 0x1B0
Longueur: 1 octet
Direction: P→S

Data[0]: Duty cycle (0=OFF, 255=pleine puissance)
```

---

## Domaine 0x2 - GROUND

Capteurs de sol (détection de bord de table).

**Target = numéro du capteur (0-2)**

### Commandes

| Cmd | Nom | Valeur |
|-----|-----|--------|
| 0x0 | Status | `GroundCmd::Status` |
| 0x1 | Value | `GroundCmd::Value` |
| 0x2 | SetThreshold | `GroundCmd::SetThreshold` |

### 0x200 - GROUND_STATUS

État de tous les capteurs de sol. Envoyé périodiquement.

```
ID: 0x200
Longueur: 1 octet
Direction: S→P

Data[0]: Bitmask de détection
         - bit 0: Capteur 0 (module 0) détecte sol
         - bit 1: Capteur 1 (module 1) détecte sol
         - bit 2: Capteur 2 (module 2) détecte sol
```

### 0x21X - GROUND_VALUE

Valeur brute d'un capteur de sol.

**Requête (P→S):**
```
ID: 0x21[capteur]
Longueur: 0 octet
```

**Réponse (S→P):**
```
ID: 0x21[capteur]
Longueur: 4 octets

Data[0-1]: Valeur brute proximity (little-endian)
Data[2-3]: Seuil de détection actuel (little-endian)
```

### 0x22X - SET_GROUND_THRESHOLD

Configure le seuil de détection.

```
ID: 0x22[capteur]
Longueur: 2 octets
Direction: P→S

Data[0-1]: Nouveau seuil (little-endian)
```

---

## Domaine 0x3 - LOG

Logs envoyés de picotter vers sabotter.

Les logs utilisent le crate `log` standard Rust. Les messages longs sont fragmentés en plusieurs trames CAN.

### Commandes

| Cmd | Nom | Valeur |
|-----|-----|--------|
| 0x0 | Config | `LogCmd::Config` |
| 0x1 | Msg | `LogCmd::Msg` |
| 0x2 | Cont | `LogCmd::Cont` |
| 0xF | End | `LogCmd::End` |

### 0x300 - LOG_CONFIG

Configure le niveau de log.

```
ID: 0x300
Longueur: 1 octet
Direction: P→S

Data[0]: Niveau de log minimum (LogLevel)
         - 0 = OFF (pas de logs)
         - 1 = ERROR
         - 2 = WARN
         - 3 = INFO
         - 4 = DEBUG
         - 5 = TRACE
```

### 0x310 - LOG_MSG

Premier fragment d'un message de log.

```
ID: 0x310
Longueur: 2-8 octets
Direction: S→P

Data[0]: Numéro de séquence (u8)
Data[1]: Niveau (LogLevel: 1=error, 2=warn, 3=info, 4=debug, 5=trace)
Data[2-7]: Payload (jusqu'à 6 bytes UTF-8)
```

### 0x320 - LOG_CONT

Fragment de continuation.

```
ID: 0x320
Longueur: 2-8 octets
Direction: S→P

Data[0]: Numéro de séquence (doit correspondre au LOG_MSG)
Data[1-7]: Payload (jusqu'à 7 bytes UTF-8)
```

### 0x3F0 - LOG_END

Dernier fragment d'un message de log.

```
ID: 0x3F0
Longueur: 2-8 octets
Direction: S→P

Data[0]: Numéro de séquence
Data[1]: Longueur totale du message
Data[2-7]: Derniers bytes du payload (0-6 bytes)
```

**Assemblage côté récepteur:**
1. Recevoir LOG_MSG, stocker seq et niveau, démarrer buffer
2. Recevoir 0+ LOG_CONT avec même seq, concaténer
3. Recevoir LOG_END, vérifier longueur, afficher log

**Message court (≤6 bytes):** Un seul LOG_END suffit (pas de MSG/CONT nécessaire).
⚠️ Dans ce cas, le niveau de log n'est pas transmis dans la trame (l'appelant doit le fournir séparément via `LogDecoder::set_level()`).

---

## Domaine 0x4 - OTA

Mise à jour firmware over-the-air via embassy-boot.

Les messages OTA partagent les mêmes commandes pour requête/réponse,
différenciés par la taille des données.

### Commandes

| Cmd | Nom | Valeur |
|-----|-----|--------|
| 0x0 | StartReady | `OtaCmd::StartReady` |
| 0x1 | DataAck | `OtaCmd::DataAck` |
| 0x2 | FinishResult | `OtaCmd::FinishResult` |
| 0xF | AbortReboot | `OtaCmd::AbortReboot` |

### 0x400 - OTA_START

Démarre une mise à jour.

```
ID: 0x400 (cmd=StartReady, target=0)
Longueur: 8 octets
Direction: P→S

Data[0-3]: Taille du firmware (u32, LE)
Data[4-7]: CRC32 du firmware (u32, LE)
```

### 0x401 - OTA_READY

Réponse à OTA_START.

```
ID: 0x401 (cmd=StartReady, target=1)
Longueur: 1 octet
Direction: S→P

Data[0]: Status (OtaReadyStatus)
         - 0 = OK, prêt à recevoir
         - 1 = Busy (OTA déjà en cours)
         - 2 = No space (firmware trop gros)
```

### 0x410 - OTA_DATA

Chunk de données firmware.

```
ID: 0x410 (cmd=DataAck, target=0)
Longueur: 8 octets
Direction: P→S

Data[0-1]: Index du chunk (u16, LE)
Data[2-7]: Données (6 bytes)
```

### 0x411 - OTA_ACK

Acquittement d'un chunk.

```
ID: 0x411 (cmd=DataAck, target=1)
Longueur: 3 octets
Direction: S→P

Data[0-1]: Index du chunk acquitté (u16, LE)
Data[2]:   Status (0=OK, 1=Error)
```

### 0x420 - OTA_FINISH

Signale la fin du transfert.

```
ID: 0x420 (cmd=FinishResult, target=0)
Longueur: 0 octet
Direction: P→S
```

### 0x421 - OTA_RESULT

Résultat de la vérification.

```
ID: 0x421 (cmd=FinishResult, target=1)
Longueur: 5 octets
Direction: S→P

Data[0]:   Status (OtaResultStatus)
           - 0 = OK
           - 1 = CRC error
           - 2 = Size mismatch
Data[1-4]: CRC32 calculé (u32, LE)
```

### 0x4F0 - OTA_ABORT

Annule l'OTA en cours.

```
ID: 0x4F0 (cmd=AbortReboot, target=0)
Longueur: 0 octet
Direction: P→S
```

### 0x4F1 - OTA_REBOOT

Redémarre pour appliquer la mise à jour.

```
ID: 0x4F1 (cmd=AbortReboot, target=1)
Longueur: 0 octet
Direction: P→S
```

**Séquence OTA complète:**
1. Sabotter → `OTA_START` (taille + CRC)
2. Picotter → `OTA_READY` (status)
3. Sabotter → `OTA_DATA` × N (chunks de 6 bytes)
4. Picotter → `OTA_ACK` pour chaque chunk
5. Sabotter → `OTA_FINISH`
6. Picotter → `OTA_RESULT` (CRC vérifié)
7. Sabotter → `OTA_REBOOT` (si OK)

**Performance:**
- 6 bytes/chunk, ~500 chunks/s avec ACK
- Débit effectif: ~3 KB/s
- Firmware 100 KB: ~33 secondes

---

## Configuration CAN

- Bitrate: 500 kbps
- Format: CAN 2.0A (11-bit ID)
- Pas de filtrage hardware (tous messages acceptés)
