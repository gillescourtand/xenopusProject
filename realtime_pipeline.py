# -*- coding: utf-8 -*-
"""
realtime_pipeline.py

Orchestration temps réel :
- acquisition ;
- tracking ;
- affichage ;
- enregistrement.

Ce module est le cœur de la nouvelle architecture temps réel.
Il sert à lancer et arrêter les différents workers qui travaillent en parallèle.
"""

import threading
import queue

from acquisition_worker import AcquisitionWorker
from tracking_worker import TrackingWorker
from result_recorder import ResultRecorder
from performance_monitor import PerformanceMonitor


class RealtimePipeline(object):
    """
    Classe centrale qui orchestre toute la chaîne temps réel.

    Elle ne fait pas directement le tracking ou l'acquisition.
    Son rôle est plutôt de connecter les différents modules entre eux :
    - acquisition_worker : récupère les frames ;
    - tracking_worker : analyse les frames ;
    - result_recorder : sauvegarde les résultats ;
    - performance_monitor : mesure les performances.
    """

    def __init__(
        self,
        ui,
        video,
        tracking_function,
        okr_provider=None,
        frame_queue_size=500,
        result_queue_size=500,
        display_queue_size=5,
        result_file_path=None
    ):
        # Référence vers l'interface principale.
        # Elle permet au pipeline d'accéder aux paramètres sélectionnés dans l'interface
        # si nécessaire, par exemple les ROI, les seuils ou l'état des boutons.
        self.ui = ui

        # Objet vidéo utilisé par l'application historique.
        # Il contient notamment la caméra Basler, les dimensions de l'image,
        # les informations d'acquisition et l'état vidéo courant.
        self.video = video

        # Fonction de tracking appelée par le TrackingWorker.
        # Dans ce projet, elle correspond généralement à tracking_adapter(),
        # qui appelle ensuite le tracking OpenCV.
        self.tracking_function = tracking_function

        # Fournisseur de l'état OKR.
        # Il permet de récupérer l'état de la stimulation optocinétique
        # au moment de l'acquisition ou du traitement d'une frame.
        self.okr_provider = okr_provider

        # Chemin du fichier CSV de sortie.
        # Si aucune valeur n'est donnée, ResultRecorder génère un nom automatiquement.
        self.result_file_path = result_file_path

        # Événement partagé entre les threads.
        # Quand stop_event est activé, tous les workers savent qu'ils doivent s'arrêter.
        self.stop_event = threading.Event()

        # File d'attente contenant les frames à analyser.
        # AcquisitionWorker y place les FramePacket.
        # TrackingWorker y récupère les frames dans l'ordre.
        self.frame_queue = queue.Queue(maxsize=frame_queue_size)

        # File d'attente contenant les résultats de tracking.
        # TrackingWorker y place les TrackingResult.
        # ResultRecorder les récupère pour écrire le CSV.
        self.result_queue = queue.Queue(maxsize=result_queue_size)

        # File d'attente utilisée pour l'affichage.
        # Elle peut être plus petite, car l'affichage n'a pas besoin de montrer
        # toutes les frames à 200 fps. Il suffit d'afficher les dernières images.
        self.display_queue = queue.Queue(maxsize=display_queue_size)

        # Module de mesure des performances.
        # Il permet de suivre les FPS, les temps de traitement,
        # les frames manquantes et la taille des buffers.
        self.monitor = PerformanceMonitor()

        # Worker responsable de l'acquisition.
        # Il récupère les frames depuis le système d'acquisition existant
        # et les place dans frame_queue pour le tracking.
        self.acquisition_worker = AcquisitionWorker(
            ui=ui,
            video=video,
            frame_queue=self.frame_queue,
            display_queue=self.display_queue,
            stop_event=self.stop_event,
            okr_provider=okr_provider,
            monitor=self.monitor,
            display_every_n_frames=1,
        )

        # Worker responsable du tracking.
        # Il récupère les FramePacket dans frame_queue,
        # applique la fonction de tracking,
        # puis produit des TrackingResult.
        self.tracking_worker = TrackingWorker(
            frame_queue=self.frame_queue,
            result_queue=self.result_queue,
            stop_event=self.stop_event,
            tracking_function=tracking_function,
            monitor=self.monitor,
        )

        # Worker responsable de l'enregistrement CSV.
        # Il lit les TrackingResult dans result_queue
        # et écrit une ligne par résultat analysé.
        self.result_recorder = ResultRecorder(
            result_queue=self.result_queue,
            stop_event=self.stop_event,
            file_path=result_file_path,
            flush_every=50,
        )

        # Dernier résultat de tracking disponible.
        # Il est utilisé par l'affichage pour mettre à jour les overlays
        # sans attendre ou bloquer le tracking.
        self.latest_result = None

        # Verrou utilisé pour protéger l'accès à latest_result.
        # Comme plusieurs threads peuvent lire ou écrire cette variable,
        # il faut éviter les accès concurrents non maîtrisés.
        self._latest_lock = threading.Lock()

        # Thread prévu initialement pour recopier ou surveiller les résultats.
        # Dans la version actuelle, il n'est pas utilisé car AppController
        # met directement à jour latest_result via set_latest_result().
        self._result_mirror_thread = threading.Thread(target=self._mirror_results)

        # Permet au thread miroir de ne pas bloquer la fermeture de l'application.
        self._result_mirror_thread.daemon = True

        # Indique si le pipeline est actuellement actif.
        # Cela évite de lancer deux fois le pipeline en même temps.
        self.running = False

    def start(self):
        """
        Démarre la chaîne temps réel.

        Cette méthode lance :
        - le worker d'acquisition ;
        - le worker de tracking ;
        - le worker d'enregistrement.
        """

        # Si le pipeline est déjà lancé, on ne le relance pas.
        if self.running:
            return

        # Remise à zéro du signal d'arrêt.
        # Les workers peuvent donc tourner normalement.
        self.stop_event.clear()

        # Réinitialisation des statistiques de performance.
        self.monitor.reset()

        # Démarrage du thread d'acquisition.
        self.acquisition_worker.start()

        # Démarrage du thread de tracking.
        self.tracking_worker.start()

        # Démarrage du thread d'enregistrement CSV.
        self.result_recorder.start()

        # Marque le pipeline comme actif.
        self.running = True

    def stop(self, timeout=1.0):
        """
        Arrête proprement la chaîne temps réel.

        Le stop_event est activé pour demander aux workers de s'arrêter.
        Ensuite, on attend leur arrêt avec join().
        """

        # Signale à tous les workers qu'ils doivent s'arrêter.
        self.stop_event.set()

        # Attend l'arrêt des workers principaux.
        for worker in [
            self.acquisition_worker,
            self.tracking_worker,
            self.result_recorder
        ]:
            try:
                worker.join(timeout)
            except RuntimeError:
                pass

        # Marque le pipeline comme arrêté.
        self.running = False

    def set_latest_result(self, result):
        """
        Met à jour le dernier résultat disponible.

        Cette méthode est utilisée par AppController après chaque frame analysée.
        Elle permet à l'affichage d'accéder rapidement au dernier résultat
        sans consommer la file result_queue utilisée par le CSV.
        """

        with self._latest_lock:
            self.latest_result = result

    def get_latest_result(self):
        """
        Retourne le dernier résultat disponible.

        Utilisé principalement par l'affichage pour mettre à jour :
        - les overlays ;
        - les ellipses des yeux ;
        - les marqueurs de queue.
        """

        with self._latest_lock:
            return self.latest_result

    def _mirror_results(self):
        """
        Méthode prévue pour une éventuelle duplication des résultats.

        Actuellement, elle n'est pas utilisée car les résultats sont déjà :
        - envoyés au CSV via result_queue ;
        - stockés comme dernier résultat via set_latest_result().
        """

        pass

    def get_stats(self):
        """
        Retourne les statistiques courantes du pipeline.

        Permet de surveiller :
        - l'état du pipeline ;
        - la taille des files ;
        - l'état des workers ;
        - les performances mesurées.
        """

        return {
            "running": self.running,

            # Taille actuelle des files d'attente.
            # Utile pour voir si le tracking ou l'enregistrement prend du retard.
            "queues": {
                "frame_queue": self.frame_queue.qsize(),
                "result_queue": self.result_queue.qsize(),
                "display_queue": self.display_queue.qsize(),
            },

            # Statistiques du worker d'acquisition.
            "acquisition": self.acquisition_worker.get_stats(),

            # Statistiques du worker de tracking.
            "tracking": self.tracking_worker.get_stats(),

            # Statistiques du module d'enregistrement CSV.
            "recorder": self.result_recorder.get_stats(),

            # Résumé global des performances.
            "monitor": self.monitor.get_summary(),
        }