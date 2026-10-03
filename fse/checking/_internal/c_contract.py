"""Pure helpers for extracting C declarations from a mixed prose/API document."""
import re

def strip_c_comments(text: str) -> str:
    token = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/')
    def replace(match):
        value=match.group(0)
        return ''.join('\n' if c=='\n' else ' ' for c in value) if value.startswith(('/',)) else value
    return token.sub(replace,text)

def render_frozen_c_api(api_spec: str) -> str:
    lines=strip_c_comments(api_spec).splitlines()
    extracted=[]; index=0
    function_start=re.compile(r'^\s*(?!typedef\b|extern\b|#)(?:const\s+)?(?:struct\s+)?[A-Za-z_]\w*(?:\s+[A-Za-z_]\w*)*\s*\**\s+[A-Za-z_]\w*\s*\(')
    while index<len(lines):
        line=lines[index]; stripped=line.strip()
        if stripped.startswith(('#include ', '#define ')):
            extracted.append(stripped)
            while extracted[-1].endswith('\\') and index+1<len(lines):
                index+=1; extracted.append(lines[index].strip())
            index+=1;continue
        if not (stripped.startswith(('typedef ','extern ')) or function_start.match(stripped)):
            index+=1;continue
        block=[line.rstrip()]; depth=line.count('{')-line.count('}')
        while not (depth==0 and block[-1].rstrip().endswith(';')):
            index+=1
            if index>=len(lines): raise ValueError('Unterminated declaration in frozen API')
            line=lines[index];block.append(line.rstrip());depth+=line.count('{')-line.count('}')
        extracted.extend(block);index+=1
    return '#ifndef FSE_FROZEN_API_H\n#define FSE_FROZEN_API_H\n'+ '\n'.join(extracted)+'\n#endif\n'
