"""Render the current six-task means as a line chart; no intervals or footer."""
from pathlib import Path
import math

COLORS = ['#7A7F87', '#8E6CBB', '#2678B2', '#2A9D8F', '#D62728']
DASHES = [[6, 3], [6, 2, 1.5, 2], None, [2, 2], None]
LABELS = ['GPIO\noutput', 'Button\ndebounce', 'Balance\ncontrol', 'DiscoBot', 'OnStep', 'Crazyflie']


def plot(rows, data, protocol, output):
    try:
        from reportlab import rl_config
        from reportlab.graphics import renderPDF, renderSVG
        from reportlab.graphics.shapes import Circle, Drawing, Line, Polygon, Rect, String
        from reportlab.lib.colors import HexColor, white
    except ImportError as exc:
        raise RuntimeError('Plotting needs ReportLab; numeric reconstruction uses only the standard library.') from exc
    rl_config.invariant = 1

    def marker(d, x, y, i, color):
        common = dict(strokeColor=color, strokeWidth=1.2, fillColor=white)
        if i == 0:
            d.add(Circle(x, y, 4.7, **common))
        elif i == 1:
            d.add(Rect(x-3.6, y-3.6, 7.2, 7.2, **common))
        elif i == 2:
            d.add(Polygon([x, y+4.6, x-4.0, y-3.3, x+4.0, y-3.3], **common))
        elif i == 3:
            d.add(Polygon([x, y+4.5, x-4.5, y, x, y-4.5, x+4.5, y], **common))
        else:
            vertices = []
            for j in range(10):
                a = math.pi/2 + j*math.pi/5
                radius = 5.4 if j % 2 == 0 else 2.3
                vertices.extend((x+radius*math.cos(a), y+radius*math.sin(a)))
            d.add(Polygon(vertices, strokeColor=color, strokeWidth=.85, fillColor=color))

    tasks, methods = data['task_order'], data['methods']
    lookup = {(r['task'], r['method']): r for r in rows}
    if len(tasks) != 6 or len(methods) != 5 or len(lookup) != 30:
        raise ValueError('The current chart requires six tasks and five methods.')
    d = Drawing(620, 293)
    left, right, bottom, top = 49, 604, 48, 233
    xs = [left+17+j*(right-left-34)/5 for j in range(6)]
    y = lambda v: bottom + (top-bottom)*v/105
    for value in (0, 20, 40, 60, 80, 100):
        yy = y(value)
        d.add(Line(left, yy, right, yy, strokeColor=HexColor('#DCE1E6'), strokeWidth=.6))
        d.add(String(left-8, yy-3.4, str(value), fontName='Helvetica', fontSize=10, textAnchor='end'))
    for x1,y1,x2,y2 in [(left,bottom,left,top),(left,bottom,right,bottom)]:
        d.add(Line(x1,y1,x2,y2,strokeColor=HexColor('#545B63'),strokeWidth=.8))
    d.add(String(left,247,'Static support (%)',fontName='Helvetica-Bold',fontSize=11.5))
    geometry = []
    for i,method in enumerate(methods):
        values = [lookup[t,method]['mean_percent'] for t in tasks]
        color = HexColor(COLORS[i])
        for j in range(5):
            style = dict(strokeColor=color, strokeWidth=2.3 if method=='EmbedDev' else 1.4)
            if DASHES[i]:
                style['strokeDashArray'] = DASHES[i]
            d.add(Line(xs[j],y(values[j]),xs[j+1],y(values[j+1]),**style))
        for task,x,value in zip(tasks,xs,values):
            if not 0 <= value <= 100:
                raise ValueError('Chart score outside 0 to 100.')
            marker(d,x,y(value),i,color)
            geometry.append({'task':task,'method':method,'mean_percent':value,'x':x,'y':y(value)})
    for x,label in zip(xs,LABELS):
        for i,line in enumerate(label.split('\n')):
            d.add(String(x,31-i*11.5,line,fontName='Helvetica',fontSize=10.5,textAnchor='middle'))
    for i,(method,x) in enumerate(zip(methods,(23,145,254,359,469))):
        color=HexColor(COLORS[i])
        style=dict(strokeColor=color,strokeWidth=2.3 if i==4 else 1.4)
        if DASHES[i]:
            style['strokeDashArray']=DASHES[i]
        d.add(Line(x,277,x+23,277,**style))
        marker(d,x+11.5,277,i,color)
        d.add(String(x+30,273.5,method+('*' if i in (1,2,3) else ''),
                     fontName='Helvetica-Bold' if i==4 else 'Helvetica',fontSize=10.8))
    output=Path(output)
    renderPDF.drawToFile(d,str(output/'rq3_static_support.pdf'))
    renderSVG.drawToFile(d,str(output/'rq3_static_support.svg'))
    return {'chart_type':'line_means_only','point_count':30,'connecting_segments':25,
            'error_bars':0,'confidence_bands':0,'footer_present':False,
            'colors':dict(zip(methods,COLORS)),'geometry':geometry}
