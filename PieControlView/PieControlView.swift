//
//  PieControlView.swift
//
//  Created by David Switzer on 7/9/16.
//  Copyright © 2016, all rights reserved.
//

import UIKit

@IBDesignable class PieControlView : UIView{
    @IBInspectable var fullRangeValue : Float = 1.0{
        didSet{
            self.setNeedsDisplay();
        }
    }
    
    @IBInspectable var currentValue : Float = 0.5{
        didSet{
            self.setNeedsDisplay();
        }
    }
    
    @IBInspectable var strokeColor : UIColor = UIColor.whiteColor(){
        didSet{
            self.setNeedsDisplay();
        }
    }
    
    @IBInspectable var fillColor : UIColor = UIColor.whiteColor(){
        didSet{
            self.setNeedsDisplay();
        }
    }
    
    @IBInspectable var innerRadiusRatio : CGFloat = 0.75{
        didSet{
            self.setNeedsDisplay();
        }
    }
    
    @IBInspectable var outerRingWidth : CGFloat = 4.0{
        didSet{
            self.setNeedsDisplay();
        }
    }
    
    override init(frame: CGRect) {
        super.init(frame: frame)
    }
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
    }
    
    override func drawRect(rect: CGRect) {
        let center : CGPoint = CGPoint(x: bounds.origin.x + bounds.size.width / 2.0, y: bounds.origin.y + bounds.size.height / 2.0);
        
        var outerRadius : CGFloat;
        if (bounds.size.width > bounds.size.height){
            outerRadius = (bounds.size.width / 2.0) - outerRingWidth;
        }
        else{
            outerRadius = (bounds.size.height / 2.0) - outerRingWidth;
        }
        
        let outerRingPath : UIBezierPath = UIBezierPath(arcCenter: center, radius: outerRadius, startAngle: 0.0, endAngle: CGFloat(2.0 * M_PI), clockwise: true);
        fillColor.setStroke();
        outerRingPath.lineWidth = outerRingWidth;
        outerRingPath.stroke();
        
        let innerRadius : CGFloat = innerRadiusRatio * outerRadius;
        let startAngle = -CGFloat(M_PI_2);
        let endAngle = startAngle + (CGFloat(2.0 * M_PI) * CGFloat(currentValue / fullRangeValue));
        
        let timeArcPath = UIBezierPath(arcCenter: center, radius: innerRadius, startAngle: startAngle, endAngle: endAngle, clockwise: true);
        timeArcPath.addLineToPoint(center);
        timeArcPath.closePath();
        
        fillColor.setFill()
        timeArcPath.fill();
    }
}