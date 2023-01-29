#define AgentNeighbourDebug
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace SimpleSteering
{
    public class Common
    {
        //-------------------- LinesIntersection2D-------------------------
        //
        //	Given 2 lines in 2D space AB, CD this returns true if an 
        //	intersection occurs and sets dist to the distance the intersection
        //  occurs along AB. Also sets the 2d vector point to the point of
        //  intersection
        //----------------------------------------------------------------- 
        public static bool LineIntersection2D(Vector2 A,
                                              Vector2 B,
                                              Vector2 C,
                                              Vector2 D,
                                              float dist,
                                              ref Vector2 point)
        {
            float rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
            float rBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            float sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);
            float sBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            if ((rBot == 0) || (sBot == 0))
            {
                //lines are parallel
                return false;
            }

            float r = rTop / rBot;
            float s = sTop / sBot;

            if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
            {
                dist = Vector2.Distance(A, B) * r;

                point = A + r * (B - A);

                return true;
            }

            else
            {
                dist = 0;

                return false;
            }
        }
        public static bool LineIntersection2D(Vector2 A,
                                              Vector2 B,
                                              Vector2 C,
                                              Vector2 D)
        {
            float rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
            float rBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            float sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);
            float sBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

            if ((rBot == 0) || (sBot == 0))
            {
                //lines are parallel
                return false;
            }

            float r = rTop / rBot;
            float s = sTop / sBot;

            if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
            {
                return true;
            }

            else
            {
                return false;
            }
        }
        public static Vector2 Vec2DRotateAroundOrigin(Vector2 vector2, float angle)
        {
            var rotate = Quaternion.AngleAxis(angle, Vector3.up);

            var rotateMatrix = Matrix4x4.Rotate(rotate);

            var newfacing = rotateMatrix.MultiplyVector(new Vector3(vector2.x, 0, vector2.y));

            return new Vector2(newfacing.x, newfacing.z);
        }
        public static void TagNeighbors(BaseGameEntity entity, IEnumerable<BaseGameEntity> containerOfEntities, float radius)
        {
            //iterate through all entities checking for range
            foreach (var curEntity in containerOfEntities)
            {
                //first clear any current tag
                curEntity.UnTag();

                //work in distance squared to avoid sqrts
                Vector2 to = curEntity.Pos() - entity.Pos();

                float range = radius + curEntity.BRadius();

                //if entity within range, tag for further consideration
                if (curEntity != entity && (to.sqrMagnitude < range * range))
                {
                    curEntity.Tag();
                }
            }//next entity
        }
        public static void EnforceNonPenetrationConstraint(BaseGameEntity entity, IEnumerable<BaseGameEntity> containerOfEntities)
        {
            foreach (var curEntity in containerOfEntities)
            {
                if (curEntity == entity) continue;

                Vector2 toEntity = entity.Pos() - curEntity.Pos();

                float DistFromEachOther = Vector2.Distance(entity.Pos(), curEntity.Pos());

                float AmountOfOverLap = curEntity.BRadius() + entity.BRadius() - DistFromEachOther;

                if (AmountOfOverLap >= 0)
                {
                    if (DistFromEachOther != 0)
                        entity.SetPos(entity.Pos() + (toEntity / DistFromEachOther) * AmountOfOverLap);
                    else
                        entity.SetPos(entity.Pos() + (new Vector2(Random.value, Random.value) / (0.5f)) * AmountOfOverLap);
                }
            }
        }
        public static void EnforceNonPenetrationOnWall(BaseGameEntity entity, IEnumerable<Wall2D> containerOfwalls)
        {
            foreach (var wall in containerOfwalls)
            {
                float distanceToLine = PointToLineDistance(entity.Pos(), wall.From(), wall.To());
                float AmountOfOverLap = entity.BRadius() - distanceToLine;

                if (AmountOfOverLap >= 0 && LineIntersection2D(wall.From(), wall.To(), entity.Pos(), entity.Pos() - wall.Normal() * entity.BRadius() * 2))
                {
                    if (distanceToLine != 0)
                        entity.SetPos(entity.Pos() + (wall.Normal()) * AmountOfOverLap);
                    else
                        entity.SetPos(entity.Pos() + (new Vector2(Random.value, Random.value) / (0.5f)) * AmountOfOverLap);
                }
            }
        }
        public static float PointToLineDistance(Vector2 point, Vector2 line_p1, Vector2 line_p2)
        {
            /// This is the formula of distance point to line
            float distance = Mathf.Abs((line_p2.x - line_p1.x) * (line_p1.y - point.y) - (line_p1.x - point.x) * (line_p2.y - line_p1.y))
                            / Mathf.Sqrt((line_p2.x - line_p1.x) * (line_p2.x - line_p1.x) + (line_p2.y - line_p1.y) * (line_p2.y - line_p1.y));

            return distance;
        }
        public static bool AccumulateForce(ref Vector2 steeringforce, Vector2 ForceToAdd, float MaxForce)
        {
            // 启发函数 帮助节省计算以及 包含一个优先级的考量
            float forceMagnitudeSofar = steeringforce.magnitude;

            float forceMagnitudeRemaining = MaxForce - forceMagnitudeSofar;

            if (forceMagnitudeRemaining <= 0) return false;

            float forceMagnitutdeToAdd = ForceToAdd.magnitude;

            if (forceMagnitutdeToAdd < forceMagnitudeRemaining) steeringforce += ForceToAdd;
            else steeringforce += ForceToAdd.normalized * forceMagnitudeRemaining;

            return true;
        }
        public static void TagObstaclesWithinViewRange(BaseGameEntity m_pVehicle, float m_dDBoxLength)
        {
            TagNeighbors(m_pVehicle, AgentManager.Instance.Obstacles(), m_dDBoxLength);
        }
        public static void TagVehiclesWithinViewRange(BaseGameEntity vehicle, float viewDistance)
        {
            Common.TagNeighbors(vehicle, AgentManager.Instance.Vehicles(), viewDistance);
        }
        public static void TagVehiclesWithinViewRangeForFLocking(BaseGameEntity vehicle, float viewDistance)
        {
            List<BaseGameEntity> list = new List<BaseGameEntity>();
            foreach (var entity in AgentManager.Instance.Vehicles())
            {
                if ((entity.Steering().isCohesionOn() || entity.Steering().isSeparationOn() || entity.Steering().isAlignmentOn())
                    && entity.GetEntityType() == vehicle.GetEntityType())
                    list.Add(entity);
            }
            Common.TagNeighbors(vehicle, list, viewDistance);
        }
        public static bool IsPosInsideObstacles(Vector2 pos) 
        {
            if (AgentManager.Instance.Obstacles().Count == 0)
                return false;

            foreach (var obstcle in AgentManager.Instance.Obstacles())
            {
                if(pos.x < obstcle.Pos().x + obstcle.BRadius() && pos.x > obstcle.Pos().x - obstcle.BRadius())
                {
                    if(pos.y < obstcle.Pos().y + obstcle.BRadius() && pos.y > obstcle.Pos().y - obstcle.BRadius()){
                        return true;
                    }
                }
            }
            return false;
        }
        public static bool IsPosOutsideGameWorld()
        {
            return false;
        }
    }
}