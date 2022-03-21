import math

import plotly.graph_objects as go
import dash
from dash import dcc, html, Input, Output
import joint_defs


# joint_fig = go.Figure()
# actuator_fig = go.Figure()
# force_fig = go.Figure()
#
#
#
# global_fig = go.Figure()

def plot_all() -> go.Figure():
    fig = go.Figure()
    fig.add_scatter3d(
        name="",
        visible=True,
        showlegend=False,
        opacity=0,
        hoverinfo='none',
        x=[-700, 700],
        y=[-700, 700],
        z=[-1400, 0]
    )
    fig.update_layout(
        width=800,
        height=600,
        scene=dict(
            # camera=dict(
            #     up=dict(x=0, y=0, z=0),
            # ),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=1)
        )
    )
    for joint in joint_defs.ALL_JOINTS:
        joint.plot_plotly(fig)
    return fig


fig = plot_all()

app = dash.Dash()
app.layout = html.Div([
    dcc.Graph(id='graph', figure=fig),
    dcc.Slider(-90, 90, 1, id='slider', value=0, marks=None, updatemode='drag', tooltip={'placement': 'bottom', 'always_visible': True})
])


@app.callback(
    Output('graph', 'figure'),
    Input('slider', 'value')
)
def update_graph(value):
    app.logger.info(f"Value: {value}")
    joint_defs.RL4_KNE_PIT.angles[0] = math.radians(value)
    return plot_all()


if __name__ == '__main__':
    app.run_server(debug=True)
