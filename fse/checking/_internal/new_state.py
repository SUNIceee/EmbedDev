# new_state.py
from typing import TypedDict, List, Dict, Any, NotRequired

class NewGraphState(TypedDict):
    user_requirement: str
    device_interface: str
    api_spec: str
    target_language: str
    
    state_diagram: str
    state_model: Dict[str, Any]
    function_skeleton: str
    interface_model: Dict[str, Any]
    sequence_diagram: str
    behavior_model: Dict[str, Any]
    model_validation_report: Dict[str, Any]
    verification_spec: List[Dict]
    validation_report: str
    
    generated_header: str
    generated_code: str
    readme_content: str
    
    tlc_verification_report: Dict
    design_model_tla: str
    design_model_cfg: str
    validation_retries: int
    max_retries: int
    
    # Consistency-check fields
    consistency_report: str
    error_context: str

    # Implementation Verifier fidelity-audit fields
    fidelity_report: str
    code_gen_retries: int

    # Deterministic orchestration metadata.
    build_profile: Dict[str, Any]
    stage_status: Dict[str, str]
    artifact_manifest: Dict[str, Any]
    repair_history: List[Dict[str, Any]]
    error_history: List[Dict[str, Any]]
    error_notebook_top_k: int

    # Implementation Generator candidate/release outcome metadata.
    run_status: NotRequired[str]
    artifact_status: NotRequired[str]
    publication_status: NotRequired[str]
    candidate_output: NotRequired[str]
    published_output: NotRequired[str]
    selected_attempt: NotRequired[int]
    selected_candidate_score: NotRequired[List[int]]
    previous_generated_header: NotRequired[str]
    previous_generated_code: NotRequired[str]
    # The requirement-model-code trace used during design repair and final audit.
    traceability: NotRequired[Dict[str, Any]]
