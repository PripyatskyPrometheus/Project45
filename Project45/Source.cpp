#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>
using namespace std;

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex begin, end;
        Distance distance;
    };
private:
    unordered_map<Vertex, vector<Edge>> _data;

    void walk_(unordered_map<Vertex, bool>& visited, const Vertex& vertex, vector<Vertex>& result) {
        visited[vertex] = true;
        result.push_back(vertex);
        for (const auto& edge : _data[vertex]) {
            const auto& next_vertex = edge.end;
            if (!visited[next_vertex]) {
                walk_(visited, next_vertex, result);
            }
        }
    }

    void walk_for_task(Vertex vert, vector<bool>& visited, Distance& sum_length, size_t& num_neighbors) {
        visited[vert] = true;
        for (const auto& edge : _data[vert]) {
            Vertex neighbor = edge.end;
            Distance len = edge.distance;
            if (!visited[neighbor]) {
                sum_length += len;
                num_neighbors++;
                walk_for_task(neighbor, visited, sum_length, num_neighbors);
            }
        }
    }
public:
    Graph() {}

    ~Graph() {
        if (!_data.empty()) _data.clear();
    }

    friend ostream& operator<<(ostream& out, const Edge& edge) {
        out << "(begin: " << edge.begin << "; end: " << edge.end << "; distance: " << edge.distance << ")";
        return out;
    }

    bool has_vertex(const Vertex& v) const {
        for (const auto& vert : _data) {
            if (v == vert.first) return true;
        }
        return false;
    }

    void add_vertex(const Vertex& v) {
        _data.insert({ v, {} });
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v)) return false;
        _data.erase(v);
        for (auto& iter : _data) {
            for (auto iter2 = iter.second.begin(); iter2 != iter.second.end();) {
                if ((*iter2).end == v) iter2 = iter.second.erase(iter2);
                else iter2++;
            }
        }
        return true;
    }

    unique_ptr<vector<Vertex>> vertices() const {
        vector<Vertex> result;
        for (auto& i : _data) {
            result.push_back(i.first);
        }
        return make_unique<vector<Vertex>>(result);
    }

    bool has_edge(const Vertex& begin, const Vertex& end) const {
        if (!has_vertex(begin) || !has_vertex(end)) return false;
        auto& edges = _data.find(begin)->second;
        for (auto& edge : edges) {
            if (edge.begin == begin && edge.end == end) return true;
        }
        return false;
    }

    bool has_edge(const Edge& e) const {
        if (!has_vertex(e.begin) || !has_vertex(e.end)) return false;
        auto& edges = _data.find(e.begin)->second;
        for (auto& edge : edges) {
            if (edge.begin == e.begin && edge.end == e.end && edge.distance == e.distance) return true;
        }
        return false;
    }

    void add_edge(const Vertex& begin, const Vertex& end, const Distance& d) {
        _data.find(begin)->second.push_back({ begin, end, d });
    }

    bool remove_edge(const Vertex& begin, const Vertex& end) {
        if (!has_vertex(begin) || !has_vertex(end)) return false;
        auto& edges = _data.find(begin)->second;
        auto iter = edges.begin();
        bool flag = false;
        while (iter != edges.end()) {
            if (iter->begin == begin && iter->end == end) {
                iter = edges.erase(iter);
                flag = true;
            }
            else ++iter;
        }
        return flag;
    }

    bool remove_edge(const Edge& e) {
        if (!has_vertex(e.begin) || !has_vertex(e.end)) return false;
        auto& edges = _data.find(e.begin)->second;
        auto iter = edges.begin();
        bool flag = false;
        while (iter != edges.end()) {
            if (iter->begin == e.begin && iter->end == e.end && iter->distance == e.distance) {
                iter = edges.erase(iter);
                flag = true;
            }
            else ++iter;
        }
        return flag;
    }

    unique_ptr<vector<Edge>> edges(const Vertex& v) const {
        vector<Edge> result;
        auto& edges = _data.find(v)->second;
        auto iter = edges.begin();
        while (iter != edges.end()) {
            result.push_back(*iter);
            iter++;
        }
        return make_unique<vector<Edge>>(result);
    }

    size_t order() const {
        return _data.size();
    }

    size_t degree(const Vertex& v) const {
        return _data.find(v)->second.size();
    }

    vector<Vertex> walk(const Vertex& start_vertex) {
        unordered_map<Vertex, bool> visited;
        vector<Vertex> result;
        walk_(visited, start_vertex, result);
        return result;
    }

    unique_ptr<vector<Edge>> shortest_path(const Vertex& begin, const Vertex& end) {
        unordered_map<Vertex, Distance> distances;
        unordered_map<Vertex, Vertex> prevs;
        for (auto& v : _data) {
            distances[v.first] = numeric_limits<Distance>::infinity();
        }
        distances[begin] = 0;
        priority_queue<pair<Vertex, Distance>, vector<pair<Vertex, Distance>>, greater<pair<Vertex, Distance>>> pq;
        pq.push(make_pair(begin, 0));
        while (!pq.empty()) {
            Vertex u = pq.top().first;
            pq.pop();
            for (auto& edge : _data[u]) {
                Vertex v = edge.end;
                Distance dist = edge.distance;
                if (distances[u] + dist < distances[v]) {
                    distances[v] = distances[u] + dist;
                    pq.push(make_pair(v, distances[v]));
                    prevs[edge.end] = edge.begin;
                }
            }
        }
        vector<Vertex> path;
        Vertex cur = end;
        while (cur != begin) {
            path.push_back(cur);
            cur = prevs[cur];
        }
        path.push_back(begin);
        reverse(path.begin(), path.end());
        Distance cur_dist = 0;
        vector<Edge> result;
        cur = path[0];
        size_t i = 0;
        while (cur != end) {
            Vertex next = path.at(i);
            auto& edges = _data.find(cur)->second;
            for (auto& edge : edges) {
                if (edge.end == _data.find(next)->first && edge.distance + cur_dist == distances[next]) {
                    result.push_back(edge);
                    cur_dist += edge.distance;
                }
            }
            cur = next;
            ++i;
        }
        return make_unique<vector<Edge>>(result);
    }

    Vertex max_average_vert() {
        Distance max_average_length = numeric_limits<Distance>::min();
        Vertex result = 0;
        for (int i = 0; i < _data.size(); ++i) {
            vector<bool> visited(_data.size(), false);
            Distance sum_length = 0;
            size_t num_neighbors = 0;
            walk_for_task(i, visited, sum_length, num_neighbors);
            Distance average_length = sum_length / num_neighbors;
            if (average_length > max_average_length) {
                max_average_length = average_length;
                result = i;
            }
        }
        return result;
    }
};

void menu() {
    Graph<size_t, double> d;
    int choice;
    while (true) {
        cout << endl;
        cout << "1) add vertex" << endl;
        cout << "2) has vertex" << endl;
        cout << "3) delete vertex" << endl;
        cout << "4) add edge" << endl;
        cout << "5) has edge" << endl;
        cout << "6) delete edge" << endl;
        cout << "7) walk" << endl;
        cout << "8) shortest path" << endl;
        cout << "10) exit" << endl;
        cin >> choice;
        switch (choice) {
        case (1): {
            cout << "enter a vertex: ";
            size_t vertex;
            cin >> vertex;
            if (d.has_vertex(vertex)) {
                cout << "Vertex exists." << endl;
                break;
            }
            d.add_vertex(vertex);
            break;
        }
        case (2): {
            size_t v;
            cout << "Enter vertex number: ";
            cin >> v;
            if (d.has_vertex(v)) {
                cout << "Vertex exists." << endl;
            }
            else {
                cout << "Vertex does not exist." << endl;
            }
            break;
        }
        case (3): {
            size_t u;
            cout << "Enter vertex number to delete: ";
            cin >> u;
            if (d.remove_vertex(u)) {
                cout << "Vertex deleted." << endl;
            }
            else {
                cout << "Vertex does not exist." << endl;
            }
            break;
        }
        case (4): {
            size_t a, b;
            int w;
            cout << "Enter begin / end / distance: ";
            cin >> a >> b >> w;
            if (!d.has_vertex(a) || !d.has_vertex(b)) {
                cout << "There are no such vertices" << endl;
                break;
            }
            d.add_edge(a, b, w);

            break;
        }
        case (5): {
            size_t x, y;
            cout << "Enter vertices to check for edge: ";
            cin >> x >> y;
            if (d.has_edge(x, y)) {
                cout << "Edge exists." << endl;
            }
            else {
                cout << "Edge does not exist." << endl;
            }
            break;
        }
        case (6): {
            size_t p, q;
            cout << "Enter vertices to delete edge between: ";
            cin >> p >> q;
            if (d.remove_edge(p, q)) {
                cout << "Edge deleted." << endl;
            }
            else {
                cout << "Edge does not exist." << endl;
            }
            break;
        }
        case (7): {
            cout << "enter the vertex to start the crawl from: ";
            size_t v_for_walk;
            cin >> v_for_walk;
            auto vs = d.walk(v_for_walk);
            for (auto i : vs) {
                cout << i << " ";
            }
            break;
        }
        case (8): {
            size_t s, t;
            cout << "Enter start and end vertices: ";
            cin >> s >> t;
            auto v = d.shortest_path(s, t);
            for (auto& i : *v.get()) {
                cout << i << " ";
            }
            break;
        }
        case (9):
            return;
        default:
            cout << "Invalid choice. Please enter a number from 1 to 10." << endl;
        }
    }
}

void menu_1() {
    Graph<double, double> d;
    int choice;
    while (true) {
        cout << endl;
        cout << "1) add vertex" << endl;
        cout << "2) has vertex" << endl;
        cout << "3) delete vertex" << endl;
        cout << "4) add edge" << endl;
        cout << "5) has edge" << endl;
        cout << "6) delete edge" << endl;
        cout << "7) walk" << endl;
        cout << "8) shortest path" << endl;
        cout << "9) exit" << endl;
        cin >> choice;
        switch (choice) {
        case (1): {
            cout << "enter a vertex: ";
            double vertex;
            cin >> vertex;
            if (d.has_vertex(vertex)) {
                cout << "Vertex exists." << endl;
                break;
            }
            d.add_vertex(vertex);
            break;
        }
        case (2): {
            double v;
            cout << "Enter vertex number: ";
            cin >> v;
            if (d.has_vertex(v)) {
                cout << "Vertex exists." << endl;
            }
            else {
                cout << "Vertex does not exist." << endl;
            }
            break;
        }
        case (3): {
            double u;
            cout << "Enter vertex number to delete: ";
            cin >> u;
            if (d.remove_vertex(u)) {
                cout << "Vertex deleted." << endl;
            }
            else {
                cout << "Vertex does not exist." << endl;
            }
            break;
        }
        case (4): {
            double a, b;
            int w;
            cout << "Enter begin / end / distance: ";
            cin >> a >> b >> w;
            if (!d.has_vertex(a) || !d.has_vertex(b)) {
                cout << "There are no such vertices" << endl;
                break;
            }
            d.add_edge(a, b, w);

            break;
        }
        case (5): {
            double x, y;
            cout << "Enter vertices to check for edge: ";
            cin >> x >> y;
            if (d.has_edge(x, y)) {
                cout << "Edge exists." << endl;
            }
            else {
                cout << "Edge does not exist." << endl;
            }
            break;
        }
        case (6): {
            double p, q;
            cout << "Enter vertices to delete edge between: ";
            cin >> p >> q;
            if (d.remove_edge(p, q)) {
                cout << "Edge deleted." << endl;
            }
            else {
                cout << "Edge does not exist." << endl;
            }
            break;
        }
        case (7): {
            cout << "enter the vertex to start the crawl from: ";
            double v_for_walk;
            cin >> v_for_walk;
            auto vs = d.walk(v_for_walk);
            for (auto i : vs) {
                cout << i << " ";
            }
            break;
        }
        case (8): {
            double s, t;
            cout << "Enter start and end vertices: ";
            cin >> s >> t;
            auto v = d.shortest_path(s, t);
            for (auto& i : *v.get()) {
                cout << i << " ";
            }
            break;
        }
        case (9):
            return;
        default:
            cout << "Invalid choice. Please enter a number from 1 to 10." << endl;
        }
    }
}

void menu_2() {
    Graph<string, double> d;
    int choice;
    while (true) {
        cout << endl;
        cout << "1) add vertex" << endl;
        cout << "2) has vertex" << endl;
        cout << "3) delete vertex" << endl;
        cout << "4) add edge" << endl;
        cout << "5) has edge" << endl;
        cout << "6) delete edge" << endl;
        cout << "7) walk" << endl;
        cout << "8) shortest path" << endl;
        cout << "9) exit" << endl;
        cin >> choice;
        switch (choice) {
        case (1): {
            cout << "enter a vertex: ";
            string vertex;
            cin >> vertex;
            if (d.has_vertex(vertex)) {
                cout << "Vertex exists." << endl;
                break;
            }
            d.add_vertex(vertex);
            break;
        }
        case (2): {
            string v;
            cout << "Enter vertex number: ";
            cin >> v;
            if (d.has_vertex(v)) {
                cout << "Vertex exists." << endl;
            }
            else {
                cout << "Vertex does not exist." << endl;
            }
            break;
        }
        case (3): {
            string u;
            cout << "Enter vertex number to delete: ";
            cin >> u;
            if (d.remove_vertex(u)) {
                cout << "Vertex deleted." << endl;
            }
            else {
                cout << "Vertex does not exist." << endl;
            }
            break;
        }
        case (4): {
            string a, b;
            int w;
            cout << "Enter begin / end / distance: ";
            cin >> a >> b >> w;
            if (!d.has_vertex(a) || !d.has_vertex(b)) {
                cout << "There are no such vertices" << endl;
                break;
            }
            d.add_edge(a, b, w);

            break;
        }
        case (5): {
            string x, y;
            cout << "Enter vertices to check for edge: ";
            cin >> x >> y;
            if (d.has_edge(x, y)) {
                cout << "Edge exists." << endl;
            }
            else {
                cout << "Edge does not exist." << endl;
            }
            break;
        }
        case (6): {
            string p, q;
            cout << "Enter vertices to delete edge between: ";
            cin >> p >> q;
            if (d.remove_edge(p, q)) {
                cout << "Edge deleted." << endl;
            }
            else {
                cout << "Edge does not exist." << endl;
            }
            break;
        }
        case (7): {
            cout << "enter the vertex to start the crawl from: ";
            string v_for_walk;
            cin >> v_for_walk;
            auto vs = d.walk(v_for_walk);
            for (auto i : vs) {
                cout << i << " ";
            }
            break;
        }
        case (8): {
            string s, t;
            cout << "Enter start and end vertices: ";
            cin >> s >> t;
            auto v = d.shortest_path(s, t);
            for (auto& i : *v.get()) {
                cout << i << " ";
            }
            break;
        }
        case (9):
            return;
        default:
            cout << "Invalid choice. Please enter a number from 1 to 10." << endl;
        }
    }
}

int main() {
    int choice;
    while (true) {
        cout << "1. size_t" << endl;
        cout << "2. double" << endl;
        cout << "3. string" << endl;
        cout << "4. Task." << endl;
        cout << "5. exit" << endl;
        cin >> choice;
        if (choice == 1) {
            menu();
        }
        if (choice == 2) {
            menu_1();
        }
        if (choice == 3) {
            menu_2();
        }
        if (choice == 4) {
            Graph<size_t, double> d;
                d.add_vertex(1);
                d.add_vertex(2);
                d.add_vertex(3);
                d.add_vertex(4);
                /*auto ver = d.vertices();
                for (auto& i : *ver.get()) {
                    cout << i << " ";
                }*/
                cout << endl;
                d.add_edge(1, 2, 5);
                d.add_edge(2, 4, 6);
                d.add_edge(1, 3, 6);
                d.add_edge(3, 4, 2);
                d.add_edge(2, 3, 4);
                /*auto ed = d.edges(1);
                for (auto& i : *ed.get()) {
                    cout << i << " ";
                }
                cout << endl;*/
            auto b = d.max_average_vert();
            cout << b << endl;
        }
        if (choice == 5) {
            return 0;
        }
    }
    return 0;
}