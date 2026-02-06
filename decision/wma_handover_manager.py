import torch
from unifolm_wma.inference import WorldModelPolicy
from geometry_msgs.msg import PoseStamped

class WMAHandoverManager:
    """
    Zastępuje HandoverManager.
    WMA decyduje, czy robot powinien brać lub oddawać obiekt.
    """

    def __init__(self, checkpoint_path="/path/to/unifolm_wma_checkpoint"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.policy = WorldModelPolicy.from_pretrained(checkpoint_path, device=self.device)

    def infer_action(self, observation):
        """
        observation: dict z polami:
            - 'camera_rgb': np.array HxWx3
            - 'gripper_state': bool
            - 'human_pose': PoseStamped
            - 'human_reaching': bool
        zwraca: string akcji ['TAKE_FROM_HUMAN', 'GIVE_TO_HUMAN', 'IDLE']
        """

        # WMA wymaga tensorów / odpowiedniej reprezentacji
        obs_tensor = self._preprocess_observation(observation)

        # Planowanie sekwencji akcji na horyzoncie
        action_seq = self.policy.plan_actions(
            observation=obs_tensor,
            task="handover_decision",   # semantyczny task
            horizon=8
        )

        # Najprostszy wybór: pierwsza zaplanowana akcja
        return action_seq[0] if action_seq else "IDLE"

    def _preprocess_observation(self, observation):
        """
        Konwertuje obserwacje ROS -> tensor dla WMA
        """
        import numpy as np
        camera = observation['camera_rgb']    # HxWx3
        camera_tensor = torch.from_numpy(camera).permute(2,0,1).float().unsqueeze(0) / 255.0

        # Zakoduj gripper_state i human_reaching
        binary_features = torch.tensor([
            int(observation['gripper_state']),
            int(observation['human_reaching'])
        ]).float().unsqueeze(0)

        # Pozycję ręki człowieka można dodać jako 3D wektor
        human_pos = observation['human_pose'].pose.position
        human_tensor = torch.tensor([
            human_pos.x, human_pos.y, human_pos.z
        ]).float().unsqueeze(0)

        # Połącz wszystko w słownik zgodny z interfejsem WMA
        return {
            "camera_rgb": camera_tensor,
            "binary_features": binary_features,
            "human_pos": human_tensor
        }
