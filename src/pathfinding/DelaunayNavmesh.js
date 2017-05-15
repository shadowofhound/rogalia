/* global Point */

// const Triangulation = require('delaunay-fast')
// const _ = require('lodash')

let nextNode = 0
let nextPoint = 0

class DelaunayNavmesh {
    constructor(obstacles) {
        this.obstacles = obstacles.map(obstacle => {
            const {x, y, w, h} = obstacle
            const ps = [
                {x, y, obstacle, id: ++nextPoint},
                {x: x + w, y, obstacle, id: ++nextPoint},
                {x: x + w, y: y + h, obstacle, id: ++nextPoint},
                {x, y: y + h, obstacle, id: ++nextPoint},
            ]
            obstacle.points = ps
            obstacle.sides = [[ps[0], ps[1]], [ps[1], ps[2]], [ps[2], ps[3]], [ps[3], ps[0]]]
            return obstacle
        })
        this.nodes = this.build(this.obstacles)
    }

    build(obstacles) {
        nextPoint = 0
        const withArrPoint = ps => ps.map(p => Object.assign(p, {point: [p.x, p.y]}))
        const map = game.map.full
        const div = 16 //divide each side of a map into $div segments and add points at the end of each segement,
        //so all map will be covered by mesh
        const wstep = map.width/div
        const hstep = map.height/div
        const mapEdge = [
            ...(_.range(0, div+1).map(i => ({x: wstep*i, y: 0}))),
            ...(_.range(0, div+1).map(i => ({x: map.width, y: hstep*i}))),
            ...(_.range(0, div+1).map(i => ({x: map.width - wstep*i, y: map.height}))),
            ...(_.range(0, div+1).map(i => ({x: 0, y: map.height - hstep*i}))),
        ]
        let points = withArrPoint([...mapEdge, ...(_.flatMap(obstacles, o => o.points))])
        let intersections = []
        let mesh = []
        let safety = 5
        do {
            points = [...points, ...intersections]
            mesh = this._triangulate(points)
            intersections = withArrPoint(_.flatMap(mesh, n => n.intersections()))
            safety--
        } while(intersections.length && safety > 0)
        return mesh
    }

    _triangulate(points) {
        nextNode = 0
        this.obstacles.forEach(o => o.points.forEach(p => p.nodes = []))

        const indices = Delaunay.triangulate(points, 'point')
        return _.compact(indices.map((p, i) => {
            if((i+1) % 3 === 0 && i > 0) {
                const a = points[indices[i-2]], b = points[indices[i-1]], c = points[indices[i]]
                return new DelaunayNode({a, b, c})
            }
        }))
    }

    getNodeAt(x, y) {
        return this.nodes.find(n => n.contains(x, y)) //TODO optimize?
    }

    draw(ctx) {
        this.nodes.forEach(n => n.draw(ctx))
    }
}

class DelaunayNode {
    constructor(points) {
        this.id = ++nextNode
        const {a, b, c} = points
        this.points = [a, b, c]
        this.points.forEach(p => p.nodes = [...(p.nodes || []), this])
        this.a = a, this.b = b, this.c = c
        this.center = {x: (a.x + b.x + c.x)/3, y: (a.y + b.y + c.y)/3}
        this.sides = [[a, b], [b, c], [c, a]]
        this.obstacles = _.compact([a.obstacle, b.obstacle, c.obstacle])
    }

    contains(x, y) {
        const {a, b, c} = this
        return ptInTriangle({x, y}, a, b, c)
    }

    distanceTo(other) {
        const ret = d(this.center, other.center) //TODO use biome info?
        // console.log(this, other, ret)
        return ret
    }

    blocked() {
        const {a, b, c} = this
        const ret = a.obstacle === b.obstacle && b.obstacle === c.obstacle
        // if(ret) console.log('blocked: ', this.id)
        return ret
    }

    passableRadius(fromNode) {
        const {a, b, c} = this

        //all vertices are on the same obstacle. Not passable
        if(a.obstacle === b.obstacle && b.obstacle === c.obstacle)
            return 0

        //each vertex on different obstacle. Pass radius = side
        if(a.obstacle !== b.obstacle && b.obstacle !== c.obstacle) {
            const c = this.commonPoints(fromNode)
            return d(c[0], c[1])/2
        }

        //two vertices are on the same side. Pass radius = altitude to base of blocked side
        let b1, b2, h
        if(a.obstacle === b.obstacle)
            b1 = a, b2 = b, h = c
        if(b.obstacle === c.obstacle)
            b1 = b, b2 = c, h = a
        if(c.obstacle === a.obstacle)
            b1 = c, b2 = a, h = b

        return d(h, getSpPoint(b1, b2, h))/2
    }

    paths(radius) {
        return this.adjacent().filter(t => !t.blocked() && t.passableRadius(this) > radius)
    }

    commonPoints(other) {
        return _.intersection(this.points, other.points)
    }

    //https://github.com/nickjanssen/PatrolJS/blob/master/patrol.js#L581
    portalTo(other, radius) {
        const aList = this.points
        const bList = other.points

        let shared = [];

        _.each(aList, function (vId) {
            if (_.includes(bList, vId)) {
                shared.push(vId);
            }
        })

        if (shared.length < 2) return []

        if (_.includes(shared, aList[0]) && _.includes(shared, aList[aList.length - 1])) {
            // Vertices on both edges are bad, so shift them once to the left
            aList.push(aList.shift())
        }

        if (_.includes(shared, bList[0]) && _.includes(shared, bList[bList.length - 1])) {
            // Vertices on both edges are bad, so shift them once to the left
            bList.push(bList.shift())
        }

        // Again!
        shared = []

        _.each(aList, function (vId) {
            if (_.includes(bList, vId)) {
                shared.push(vId)
            }
        })


        // considering object radius
        const side = d(shared[0], shared[1])
        // const mid = (p1, p2) => ({x: (p1.x + p2.x)/2, y: (p1.y + p2.y)/2})
        // if(side < 50) {
        //     const point = mid(shared[0], shared[1])
        //     return [point, point]
        // }
        const h = this.passableRadius(other) // closest point to obstacle here. Either side or altitude
        const offset = (radius/h)*side

        const offsetTo  = (from, to) => {
            const offset = {x: ((to.x - from.x)/side)*radius, y: ((to.y - from.y)/side)*radius}
            return {x: from.x + offset.x, y: from.y + offset.y}
        }

        shared[0] = offsetTo(shared[0], shared[1], offset)
        shared[1] = offsetTo(shared[1], shared[0], offset)

        return shared
    }

    adjacent() {
        if(!this._adjacent)
            this._adjacent = _.flatMap(this.points, p => p.nodes).filter(n => this.commonPoints(n).length > 1)
        return this._adjacent
    }

    intersections() {
        if(!this._intersections) {
            const is = this.obstacles
                .map(o =>
                    o.sides.map(
                        os => this.sides
                            .map(ts => lineIntersection(os, ts))
                            .filter(p => !!p)
                            .map(p => Object.assign(p, {obstacle: o}))
                    )
                )
            this._intersections =  _.flatten(_.flatten(is))
        }
        return this._intersections
    }

    draw(ctx) {
        const [a, b, c] = this.points.map(p => new Point(p).toScreen())
        ctx.moveTo(a.x, a.y)
        ctx.lineTo(b.x, b.y)
        ctx.lineTo(c.x, c.y)
        ctx.lineTo(a.x, a.y)
        // const textAt = new Point(this.center).toScreen()
        // ctx.strokeText(this.id, textAt.x, textAt.y)
    }
}

function obstaclePoints(obstacle) {
    const {x, y, w, h} = obstacle
    return [
        {x, y, id: ++nextPoint},
        {x: x + w, y, id: ++nextPoint},
        {x: x + w, y: y + h, id: ++nextPoint},
        {x, y: y + h, id: ++nextPoint},
    ]
}

function d2(p1, p2) {
    return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)
}
function d(p1, p2) {
    return Math.sqrt(d2(p1, p2))
}

function lineIntersection(l1, l2) {
    const ret = checkLineIntersection(l1[0].x, l1[0].y, l1[1].x, l1[1].y, l2[0].x, l2[0].y, l2[1].x, l2[1].y)
    return (ret.onLine1 && ret.onLine2) ? ret : null
}

//http://jsfiddle.net/justin_c_rounds/Gd2S2/light/
function checkLineIntersection(line1StartX, line1StartY, line1EndX, line1EndY, line2StartX, line2StartY, line2EndX, line2EndY) {
    // if the lines intersect, the result contains the x and y of the intersection (treating the lines as infinite) and booleans for whether line segment 1 or line segment 2 contain the point
    var denominator, a, b, numerator1, numerator2, result = {
        x: null,
        y: null,
        onLine1: false,
        onLine2: false
    };
    denominator = ((line2EndY - line2StartY) * (line1EndX - line1StartX)) - ((line2EndX - line2StartX) * (line1EndY - line1StartY));
    if (denominator === 0) {
        return result;
    }
    a = line1StartY - line2StartY;
    b = line1StartX - line2StartX;
    numerator1 = ((line2EndX - line2StartX) * a) - ((line2EndY - line2StartY) * b);
    numerator2 = ((line1EndX - line1StartX) * a) - ((line1EndY - line1StartY) * b);
    a = numerator1 / denominator;
    b = numerator2 / denominator;

    // if we cast these lines infinitely in both directions, they intersect here:
    result.x = line1StartX + (a * (line1EndX - line1StartX));
    result.y = line1StartY + (a * (line1EndY - line1StartY));
    /*
     // it is worth noting that this should be the same as:
     x = line2StartX + (b * (line2EndX - line2StartX));
     y = line2StartX + (b * (line2EndY - line2StartY));
     */
    // if line1 is a segment and line2 is infinite, they intersect if:
    if (a > 0 && a < 1) {
        result.onLine1 = true;
    }
    // if line2 is a segment and line1 is infinite, they intersect if:
    if (b > 0 && b < 1) {
        result.onLine2 = true;
    }
    // if line1 and line2 are segments, they intersect if both of the above are true
    return result;
}

//http://stackoverflow.com/a/12499474
function getSpPoint(A,B,C) {
    var x1=A.x, y1=A.y, x2=B.x, y2=B.y, x3=C.x, y3=C.y;
    var px = x2-x1, py = y2-y1, dAB = px*px + py*py;
    var u = ((x3 - x1) * px + (y3 - y1) * py) / dAB;
    var x = x1 + u * px, y = y1 + u * py;
    return {x:x, y:y};
}

//http://jsfiddle.net/PerroAZUL/zdaY8/1/
function ptInTriangle(p, p0, p1, p2) {
    var A = 1/2 * (-p1.y * p2.x + p0.y * (-p1.x + p2.x) + p0.x * (p1.y - p2.y) + p1.x * p2.y);
    var sign = A < 0 ? -1 : 1;
    var s = (p0.y * p2.x - p0.x * p2.y + (p2.y - p0.y) * p.x + (p0.x - p2.x) * p.y) * sign;
    var t = (p0.x * p1.y - p0.y * p1.x + (p0.y - p1.y) * p.x + (p1.x - p0.x) * p.y) * sign;

    return s > 0 && t > 0 && (s + t) < 2 * A * sign;
}