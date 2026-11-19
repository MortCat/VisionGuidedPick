'use strict';

function renderCloud(canvas, cloud) {
    const context = canvas.getContext('2d');
    if (!context) return;
    const {width, height} = canvas;
    context.clearRect(0, 0, width, height);
    if (!cloud) return;
    const project = ([x, y, z]) => [x - y, (x + y) * .45 - z];
    const accumulated = cloud.accumulated.map(project);
    const latest = cloud.latest.map(project);
    const points = accumulated.concat(latest);
    if (!points.length) return;
    const xs = points.map(point => point[0]);
    const ys = points.map(point => point[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const scale = Math.min((width - 70) / Math.max(.01, maxX - minX),
        (height - 70) / Math.max(.01, maxY - minY));
    for (const [layer, color] of [[accumulated, '#8393a3'], [latest, '#5ce5f3']]) {
        context.fillStyle = color;
        for (const [x, y] of layer) {
            context.fillRect(width / 2 + (x - (minX + maxX) / 2) * scale,
                height / 2 + (y - (minY + maxY) / 2) * scale, 2, 2);
        }
    }
    context.font = '14px sans-serif';
    context.fillStyle = '#dbe5ed';
    context.fillText('Accumulated', 16, height - 16);
    context.fillStyle = '#5ce5f3';
    context.fillText('Latest Scan', 114, height - 16);
}
