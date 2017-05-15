/* global d2 */

//https://github.com/nickjanssen/PatrolJS/blob/master/patrol.js
//http://digestingduck.blogspot.ru/2010/03/simple-stupid-funnel-algorithm.html

function triarea2(a, b, c) {
    var ax = b.x - a.x;
    var ay = b.y - a.y;
    var bx = c.x - a.x;
    var by = c.y - a.y;
    return bx * ay - ax * by;
}

function vequal(a, b) {
    return d2(a, b) < 0.00001;
}

function Channel() {
    this.portals = [];
}

Channel.prototype.push = function (p1, p2) {
    if (p2 === undefined) p2 = p1;
    this.portals.push({
        left: p1,
        right: p2
    });
};

Channel.prototype.stringPull = function () {
    var portals = this.portals;
    var pts = [];
    // Init scan state
    var portalApex, portalLeft, portalRight;
    var apexIndex = 0,
        leftIndex = 0,
        rightIndex = 0;

    portalApex = portals[0].left;
    portalLeft = portals[0].left;
    portalRight = portals[0].right;

    // Add start point.
    pts.push(portalApex);

    for (var i = 1; i < portals.length; i++) {
        var left = portals[i].left;
        var right = portals[i].right;

        // Update right vertex.
        if (triarea2(portalApex, portalRight, right) <= 0.0) {
            if (vequal(portalApex, portalRight) || triarea2(portalApex, portalLeft, right) > 0.0) {
                // Tighten the funnel.
                portalRight = right;
                rightIndex = i;
            } else {
                // Right over left, insert left to path and restart scan from portal left point.
                pts.push(portalLeft);
                // Make current left the new apex.
                portalApex = portalLeft;
                apexIndex = leftIndex;
                // Reset portal
                portalLeft = portalApex;
                portalRight = portalApex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                // Restart scan
                i = apexIndex;
                continue;
            }
        }

        // Update left vertex.
        if (triarea2(portalApex, portalLeft, left) >= 0.0) {
            if (vequal(portalApex, portalLeft) || triarea2(portalApex, portalRight, left) < 0.0) {
                // Tighten the funnel.
                portalLeft = left;
                leftIndex = i;
            } else {
                // Left over right, insert right to path and restart scan from portal right point.
                pts.push(portalRight);
                // Make current right the new apex.
                portalApex = portalRight;
                apexIndex = rightIndex;
                // Reset portal
                portalLeft = portalApex;
                portalRight = portalApex;
                leftIndex = apexIndex;
                rightIndex = apexIndex;
                // Restart scan
                i = apexIndex;
                continue;
            }
        }
    }

    if ((pts.length === 0) || (!vequal(pts[pts.length - 1], portals[portals.length - 1].left))) {
        // Append last point to path.
        pts.push(portals[portals.length - 1].left);
    }

    this.path = pts;
    return pts;
};


const mid = (p1, p2) => ({x: (p1.x + p2.x)/2, y: (p1.y + p2.y)/2})

const edges = (path, radius) => {
    let prev = null
    return path.reduce((acc, current) => {
        const ret = prev ? [...acc, prev.portalTo(current, radius)] : acc
        prev = current
        return ret
    }, [])
}

function midedgeSmooth(path, from, to, radius) {
    return [...edges(path, radius).map(e => mid(e[0], e[1])), to]
}

function funnelSmooth(path, from, to, radius) {
    const c = new Channel()
    c.push(from)
    // edges(path, radius + 25 /* magic */).forEach(e => c.push(e[0], e[1]))
    edges(path, radius*2 + 5).forEach(e => c.push(e[0], e[1]))
    c.push(to)
    c.stringPull()
    return c.path
}
