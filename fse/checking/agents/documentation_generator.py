# agents/documentation_generator.py
from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel, Field
from typing import Dict

from runtime import NewGraphState
from runtime import llm

class ReadmeOutput(BaseModel):
    reasoning: str = Field(description="Documentation rationale: extract core requirements, organize design artifacts and summarize supplied Apalache BMC results.")
    readme_content: str = Field(description="Project README content in Markdown.")

class ReadmeGeneratorAgent:
    def __init__(self):
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", (
                "You are a professional project documentation engineer. Follow the documentation workflow. "
                "Write a complete project README from the supplied design artifacts and generated skeleton. "
                "Include an overview, an explained artifact list, supplied formal-verification results and the code structure. Never invent verification results. "
                "***Workflow:***"

                "#### Phase 1: inspect the supplied artifacts. "
                "1. Identify the project name and core functionality. "
                "2. Organize state diagrams, sequence diagrams and any supplied TLA+/Apalache artifacts. "
                "3. Extract the reported verification status and failures, if available. "

                "#### Phase 2: write the documentation. "
                "1. Write the overview. "
                "2. Explain the state and sequence diagrams. "
                "3. Summarize supplied formal-verification results in a separate section; state when unavailable. "
                "4. Explain the generated code structure. "

                "#### Phase 3: output. "
                "Use clear Markdown formatting."
            )),
            ("human", "{input}"),
        ])
        self.structured_llm = llm.with_structured_output(ReadmeOutput)
        self.chain = self.prompt | self.structured_llm

    def generate(self, state: NewGraphState) -> Dict[str, str]:
        print("---AGENT 7: GENERATING README.md (with CoT)---")
        
        tlc_report = state.get('tlc_verification_report', {})
        verification_status = tlc_report.get('status', 'N/A')

        input_text = (
            "Follow the documentation workflow to generate the project README.\n\n"
            f"**1. Original requirements:**\n{state.get('user_requirement', 'N/A')}\n\n"
            f"**2. Function skeleton:**\n{state.get('function_skeleton', 'N/A')}\n\n"
            f"**3. State/sequence diagram summary:**\n{state.get('state_diagram', 'N/A')[:50]}...\n{state.get('sequence_diagram', 'N/A')[:50]}...\n\n"
            f"**4. Verification report summary (Apalache BMC Status: {verification_status})：**\n{state.get('validation_report', 'N/A')[:200]}...\n\n"
            f"**5. Generated code summary:**\n{state.get('generated_code', 'N/A')[:200]}...\n"
            "Include an overview, design description, any supplied verification results and code structure."
        )

        result: ReadmeOutput = self.chain.invoke({"input": input_text})
        if result and result.readme_content:
            print(f"Reasoning: {result.reasoning[:50]}...")
            return {"readme_content": result.readme_content}
        return {"readme_content": "# Error: README generation failed."}

readme_generator_agent = ReadmeGeneratorAgent()