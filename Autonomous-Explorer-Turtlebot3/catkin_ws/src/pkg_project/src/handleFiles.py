import yaml

class HandleFiles:
    def save_poses(self, markers, file_path):
        # Salva le pose dei marker in un file YAML
        with open(file_path, 'w') as f:
            yaml.dump(markers, f, default_flow_style=False)
