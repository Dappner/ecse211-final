import logging
import threading
import time
from utils.sound import Song, Sound

logger = logging.getLogger("siren")


class SirenController:
    """Handles the creation and control of the siren sound."""

    def __init__(self):
        self.siren = None
        self.silence = None
        self.siren_low = None
        self.siren_high = None
        self.siren_thread = None
        self.siren_active = False
        self.create_siren()

    def create_siren(self):
        """Create the siren sound using the Sound module"""
        # Create high pitch siren sound
        # TODO: Make sound like siren.
        self.siren_high = Sound(
            duration=3,  # Shorter duration for the wailing effect
            volume=95,  # Loud volume for emergency alerts
            pitch="D6",  # High pitch note
            cutoff=0.05,  # Slight softening at start/end
            fs=16000,  # Higher sample rate for better quality
            mod_f=6,  # Faster frequency modulation for more realism
            mod_k=25,  # Stronger frequency modulation for authentic sound
            amp_f=12,  # Amplitude modulation for wobble
            amp_ka=0.4,  # Medium amplitude variation
            amp_ac=0.9  # Overall amplitude control
        )

        # Create the siren by combining high and low tones with brief silence in between
        self.siren = Song([
            self.siren_high,
        ])
        self.siren.compile()
        logger.info("Fire siren created with single modulated tone")

    def start(self):
        """Start playing the siren in a separate thread"""
        if self.siren_thread is not None and self.siren_thread.is_alive():
            logger.info("Siren already playing")
            return

        self.siren_active = True
        self.siren_thread = threading.Thread(target=self._play_siren, daemon=True)
        self.siren_thread.start()
        logger.info("Siren started")

    def stop(self):
        """Stop the siren"""
        if self.siren_thread is not None:
            self.siren_active = False
            self.siren.stop()
            logger.info("Siren stopped")

    def _play_siren(self):
        """Play the siren repeatedly until stopped"""
        while self.siren_active:
            self.siren.play()
            time.sleep(self.siren.duration)
            # The siren will automatically stop when the duration is up,
            # but we need to play it again if the siren is still active
            if not self.siren_active:
                break
