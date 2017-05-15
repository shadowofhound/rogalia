/* global DelaunayNavmesh, funnelSmooth, midedgeSmooth */


class Pathfinder {
    constructor(obstacles) {
        // console.log(JSON.stringify(obstacles))
        this.discretization = new DelaunayNavmesh(obstacles)
        // this.smooth = midedgeSmooth
        this.smooth = funnelSmooth
    }

    find(from, to, radius = 0) {
        const fromNode = this.discretization.getNodeAt(from.x, from.y)
        const toNode = this.discretization.getNodeAt(to.x, to.y)

        // console.log('making path from ', fromNode, ' to ', toNode)

        const nodes =  aStar({
            start: fromNode,
            isEnd: n => n === toNode,
            neighbor: n => n.paths(radius),
            distance: (a, b) => a.distanceTo(b),
            heuristic: (n) => n.distanceTo(toNode),
            hash: n => ""+n.id
        })

        // console.log(nodes)

        if(nodes.status !== "success")
            return null

        return this.smooth(nodes.path, from, to, radius).reverse()
    }
}