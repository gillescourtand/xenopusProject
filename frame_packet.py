# -*- coding: utf-8 -*-
"""
frame_packet.py

Structures de données échangées dans la chaîne temps réel.

Objectif :
- éviter les listes séparées non synchronisées ;
- associer chaque image à un frame_id, un timestamp, un état OKR et des métadonnées ;
- associer chaque résultat de tracking à la frame correspondante.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class FramePacket:
    """
    Représente une image acquise par la caméra.

    Un FramePacket est créé dès qu'une frame est récupérée.
    Il contient l'image brute ainsi que toutes les informations nécessaires
    pour la synchroniser avec le tracking, l'OKR et les autres données.
    """

    # Identifiant unique de la frame.
    # Il permet de vérifier que les frames sont bien traitées dans l'ordre
    # et de détecter les frames manquantes.
    frame_id: int

    # Timestamp logiciel associé à la frame.
    # Il correspond au temps de récupération de l'image côté ordinateur.
    # Il sert à synchroniser la vidéo avec les résultats de tracking.
    timestamp: float

    # Image acquise par la caméra.
    # C'est généralement un tableau NumPy contenant les pixels de la frame.
    image: Any

    # État de la stimulation OKR au moment où la frame est acquise.
    # Exemple : vitesse du stimulus, direction, mode, activation.
    okr_state: Optional[Dict[str, Any]] = None

    # Timestamp éventuellement fourni par la caméra elle-même.
    # Il peut être plus précis que le timestamp logiciel si la caméra le supporte.
    camera_timestamp: Optional[float] = None

    # Informations complémentaires associées à la frame.
    # Exemple : source de la frame, paramètres caméra, taille d'image, etc.
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TrackingResult:
    """
    Représente le résultat du tracking pour une frame donnée.

    Chaque TrackingResult correspond à une seule frame analysée.
    Cela permet d'avoir une ligne complète et cohérente dans le CSV.
    """

    # Identifiant de la frame analysée.
    # Il doit correspondre au frame_id du FramePacket d'origine.
    frame_id: int

    # Timestamp de la frame analysée.
    # Il permet de relier les valeurs mesurées au temps expérimental.
    timestamp: float

    # Angle mesuré pour le premier œil.
    # Valeur calculée à partir de l'ellipse détectée par OpenCV.
    eye1_angle: Optional[float] = None

    # Angle mesuré pour le deuxième œil.
    # Valeur calculée à partir de l'ellipse détectée par OpenCV.
    eye2_angle: Optional[float] = None

    # Position verticale du premier œil.
    # Sert à mesurer le déplacement de l'œil dans l'image.
    eye1_y: Optional[float] = None

    # Position verticale du deuxième œil.
    # Sert à mesurer le déplacement de l'œil dans l'image.
    eye2_y: Optional[float] = None

    # Angle de la queue.
    # Calculé à partir de la position détectée de la queue et de l'axe du corps.
    tail_angle: Optional[float] = None

    # Position horizontale de la queue détectée.
    # Permet de suivre la position du point de queue dans l'image.
    tail_x: Optional[float] = None

    # Position verticale de la queue détectée.
    # Permet de suivre la position du point de queue dans l'image.
    tail_y: Optional[float] = None

    # Mesures de queue par arcs R / M / C.
    # R = région droite/proximale, M = milieu, C = région gauche/caudale.
    tail_R_angle: Optional[float] = None
    tail_R_x: Optional[float] = None
    tail_R_y: Optional[float] = None

    tail_M_angle: Optional[float] = None
    tail_M_x: Optional[float] = None
    tail_M_y: Optional[float] = None

    tail_C_angle: Optional[float] = None
    tail_C_x: Optional[float] = None
    tail_C_y: Optional[float] = None

    # État de la stimulation OKR associé à cette frame.
    # Permet de savoir dans quelles conditions expérimentales
    # la mesure a été réalisée.
    okr_state: Optional[Dict[str, Any]] = None

    # Indique si le résultat de tracking est considéré comme valide.
    # True : tracking réussi.
    # False : erreur ou détection invalide.
    valid: bool = True

    # Message d'erreur éventuel.
    # Il est rempli si une erreur apparaît pendant le tracking.
    error: Optional[str] = None

    # Informations complémentaires produites par le tracking.
    # Exemple : descripteur d'ellipse, position d'overlay, valeurs intermédiaires.
    metadata: Dict[str, Any] = field(default_factory=dict)
